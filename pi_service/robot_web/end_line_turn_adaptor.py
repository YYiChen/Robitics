"""Port-5000 adaptor for the current single-white-line / red-terminal course.

The Pi is the only motor owner.  This first validation intentionally stops at
the red terminal and never turns; 90-degree target alignment is a later layer.
"""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import threading
import time

EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "end_line_turn_validation"
FAST_EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "pc_vision_adaptor_validation"
for path in (EXPERIMENT, FAST_EXPERIMENT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from end_line_logic import EndLineConfig, EndLineStopPlanner, RedEndBandDetector  # noqa: E402
from fast_line import FastLineConfig, find_fast_line, pwm_for_line  # noqa: E402


TUNING_PATH = EXPERIMENT / "end_line_web_tuning.json"
TUNING_RULES = {
    "process_fps": (float, 5.0, 60.0),
    "straight_pwm": (int, 0, 255),
    "correction_deadband": (float, 0.0, 1.0),
    "correction_gain": (float, 0.0, 1000.0),
    "minimum_correction_pwm": (int, 0, 255),
    "maximum_correction_pwm": (int, 0, 255),
    "red_channel_min": (int, 0, 255),
    "red_excess_min": (int, 0, 255),
    "red_roi_top_ratio": (float, 0.0, .8),
    "red_roi_side_ratio": (float, 0.0, .45),
    "red_min_component_area": (int, 1, 100000),
    "red_min_span_ratio": (float, .01, 1.0),
    "red_confirm_frames": (int, 1, 20),
    "red_stop_bottom_ratio": (float, .2, .99),
    "line_lost_confirm_frames": (int, 1, 20),
}


class EndLineTurnAdaptorRouteTracker:
    route_mode = "end_line_turn_adaptor"

    def __init__(self, controller, camera, publisher, gate, tuning_path: Path = TUNING_PATH) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        self._stop, self._thread, self._lock, self._tuning_lock = threading.Event(), None, threading.RLock(), threading.RLock()
        self._last_center_x, self._motor_active = None, False
        self._tuning_path = tuning_path
        self._process_fps, self._straight_pwm, self._fast_config, self._line_config = self._load_tuning()
        self._planner = EndLineStopPlanner(self._line_config)
        self._red_detector = RedEndBandDetector(self._line_config)
        self._run_log = None
        self._status = {"available": True, "running": False, "enabled": False, "mode": self.route_mode, "state": "starting", "detail": "单白线红终点 adaptor 启动中", "frame": 0, "confidence": 0.0}

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="end-line-turn-adaptor")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._stop_motor()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now()
        self._motor_active = False

    def _set_status(self, **changes) -> None:
        with self._lock:
            self._status.update(changes)

    def status_dict(self) -> dict:
        with self._lock:
            result = dict(self._status)
        result.update({"enabled": self.gate.enabled(), "tuning": self._tuning_values(), "tuning_path": str(self._tuning_path)})
        return result

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled:
            self._stop_motor()
            with self._tuning_lock:
                self._planner.reset()
        self._set_status(enabled=enabled, detail="单白线红终点自动行驶已开启" if enabled else "已暂停，电机已停止")
        return self.status_dict()

    def _tuning_values(self) -> dict:
        with self._tuning_lock:
            return {
                "process_fps": self._process_fps, "straight_pwm": self._straight_pwm,
                "correction_deadband": self._fast_config.deadband, "correction_gain": self._fast_config.correction_gain,
                "minimum_correction_pwm": self._fast_config.min_correction_pwm, "maximum_correction_pwm": self._fast_config.max_correction_pwm,
                **asdict(self._line_config),
            }

    def _load_tuning(self) -> tuple[float, int, FastLineConfig, EndLineConfig]:
        values = {
            "process_fps": 20.0, "straight_pwm": 85,
            "correction_deadband": FastLineConfig().deadband, "correction_gain": FastLineConfig().correction_gain,
            "minimum_correction_pwm": FastLineConfig().min_correction_pwm, "maximum_correction_pwm": FastLineConfig().max_correction_pwm,
            **asdict(EndLineConfig()),
        }
        try:
            stored = json.loads(self._tuning_path.read_text(encoding="utf-8")) if self._tuning_path.exists() else {}
            if isinstance(stored, dict):
                for key, value in stored.items():
                    if key in TUNING_RULES:
                        kind, minimum, maximum = TUNING_RULES[key]
                        converted = kind(value)
                        if minimum <= converted <= maximum:
                            values[key] = converted
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        return self._configs_from_values(values)

    @staticmethod
    def _configs_from_values(values: dict) -> tuple[float, int, FastLineConfig, EndLineConfig]:
        fast = replace(FastLineConfig(), deadband=values["correction_deadband"], correction_gain=values["correction_gain"], min_correction_pwm=values["minimum_correction_pwm"], max_correction_pwm=values["maximum_correction_pwm"])
        line = EndLineConfig(**{key: values[key] for key in asdict(EndLineConfig())})
        return values["process_fps"], values["straight_pwm"], fast, line

    def update_tuning(self, payload: dict) -> dict:
        if not isinstance(payload, dict):
            raise ValueError("调参内容必须是 JSON 对象")
        current = self._tuning_values()
        for key, raw_value in payload.items():
            if key not in TUNING_RULES:
                continue
            kind, minimum, maximum = TUNING_RULES[key]
            try:
                value = kind(raw_value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{key} 不是有效数值") from exc
            if not minimum <= value <= maximum:
                raise ValueError(f"{key} 必须在 {minimum} 到 {maximum} 之间")
            current[key] = value
        if current["minimum_correction_pwm"] > current["maximum_correction_pwm"]:
            raise ValueError("最小修正 PWM 不能大于最大修正 PWM")
        process_fps, straight_pwm, fast_config, line_config = self._configs_from_values(current)
        self._tuning_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._tuning_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(current, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self._tuning_path)
        with self._tuning_lock:
            self._process_fps, self._straight_pwm = process_fps, straight_pwm
            self._fast_config, self._line_config = fast_config, line_config
            self._red_detector, self._planner = RedEndBandDetector(line_config), EndLineStopPlanner(line_config)
        return self.status_dict()

    def _open_log(self) -> None:
        directory = EXPERIMENT / "runtime_logs"
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._run_log = (directory / f"end_line_turn_{stamp}.jsonl").open("a", encoding="utf-8")

    def _write_log(self, frame: int, result, red, decision, state: str, motor: str, commanded) -> None:
        if self._run_log is None:
            return
        controller_status = self.controller.status()
        self._run_log.write(json.dumps({
            "time_utc": datetime.now(timezone.utc).isoformat(), "kind": "end_line_cycle", "frame": frame,
            "white_line": {"valid": result.valid, "center_x": result.center_x, "confidence": result.confidence, "rows": result.centers},
            "red_terminal": asdict(red), "planner": {"state": decision.state.value, "reason": decision.reason},
            "gate_enabled": self.gate.enabled(), "state": state, "motor": motor,
            "commanded_pwm": None if commanded is None else {"right": commanded[0], "left": commanded[1]},
            "motor_output": controller_status.get("motor_output"),
        }, ensure_ascii=False) + "\n")
        self._run_log.flush()

    def _run(self) -> None:
        import cv2
        import numpy as np

        last, frame_index = 0.0, 0
        self._open_log()
        self._set_status(running=True, state="ready", detail="仅白线跟随与红终点停车；按 M 开始")
        try:
            while not self._stop.is_set():
                now, jpeg = time.monotonic(), self.camera.latest_jpeg()
                with self._tuning_lock:
                    process_fps = self._process_fps
                if jpeg is None or now - last < 1.0 / process_fps:
                    self._stop.wait(.003)
                    continue
                last = now
                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                with self._tuning_lock:
                    fast_config, detector, planner, straight_pwm = self._fast_config, self._red_detector, self._planner, self._straight_pwm
                    result = find_fast_line(image, self._last_center_x, fast_config)
                    red = detector.detect(image)
                    decision = planner.step(line_valid=result.valid, red_band=red, frame_height=image.shape[0])
                if result.center_x is not None:
                    self._last_center_x = result.center_x
                state, motor, commanded = decision.state.value, "PAUSED", None
                if self.gate.enabled():
                    if decision.stop:
                        self._stop_motor()
                        motor = "STOP_RED_TERMINAL" if red.detected else "STOP_LINE_LOST"
                    else:
                        commanded = pwm_for_line(result, image.shape[1], straight_pwm, fast_config)
                        if commanded is None:
                            self._stop_motor()
                            state, motor = "STOPPED_UNSAFE_LINE_LOST", "STOP_NO_NEAR_WHITE_LINE"
                        else:
                            self.controller.set_direct_drive(*commanded)
                            self._motor_active = True
                            motor = f"FOLLOW_PWM R={commanded[0]} L={commanded[1]}"
                else:
                    self._stop_motor()
                    with self._tuning_lock:
                        self._planner.reset()
                        decision = self._planner.step(line_valid=result.valid, red_band=red, frame_height=image.shape[0])
                    state = "PAUSED"
                overlay = image.copy()
                for y, x, _width in result.centers:
                    cv2.circle(overlay, (int(x), y), 5, (0, 255, 0), -1)
                if red.detected and red.y is not None and red.bottom_y is not None:
                    cv2.line(overlay, (0, red.y), (overlay.shape[1] - 1, red.y), (0, 0, 255), 2)
                    cv2.line(overlay, (0, red.bottom_y), (overlay.shape[1] - 1, red.bottom_y), (0, 80, 255), 1)
                cv2.rectangle(overlay, (10, 10), (1110, 112), (20, 20, 20), cv2.FILLED)
                cv2.putText(overlay, f"END-LINE ADAPTOR: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 0) if self.gate.enabled() else (0, 180, 255), 2)
                cv2.putText(overlay, f"WHITE: valid={result.valid} centre={result.center_x} conf={result.confidence:.2f}  RED END: {red.detected} y={red.y} bottom={red.bottom_y} span={red.span}", (18, 66), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
                cv2.putText(overlay, f"STATE: {state}  {decision.reason}  MOTOR: {motor}", (18, 94), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 255, 255), 1)
                ok, encoded = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                if self.gate.enabled() or frame_index % 20 == 0:
                    self._write_log(frame_index, result, red, decision, state, motor, commanded)
                self._set_status(state=state, detail=decision.reason, frame=frame_index, confidence=result.confidence, line_center_x=result.center_x, red_terminal=asdict(red), motor=motor)
                frame_index += 1
        except Exception as exc:
            import traceback
            traceback.print_exc()
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            if self._run_log is not None:
                self._run_log.close()
            self._set_status(running=False)
