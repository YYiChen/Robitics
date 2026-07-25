"""Port-5000 adaptor for the current single-white-line / red-terminal course.

The Pi is the only motor owner.  This first validation intentionally stops at
the red terminal and never turns; 90-degree target alignment is a later layer.
"""
from __future__ import annotations

from dataclasses import asdict
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


class EndLineTurnAdaptorRouteTracker:
    route_mode = "end_line_turn_adaptor"

    def __init__(self, controller, camera, publisher, gate) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        self._stop, self._thread, self._lock = threading.Event(), None, threading.RLock()
        self._last_center_x, self._motor_active = None, False
        self._planner = EndLineStopPlanner()
        self._red_detector = RedEndBandDetector()
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
        result.update({"enabled": self.gate.enabled(), "line_config": asdict(EndLineConfig())})
        return result

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled:
            self._stop_motor()
            self._planner.reset()
        self._set_status(enabled=enabled, detail="单白线红终点自动行驶已开启" if enabled else "已暂停，电机已停止")
        return self.status_dict()

    def update_tuning(self, _payload: dict) -> dict:
        raise ValueError("单白线红终点 adaptor 的参数暂不在网页修改")

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

        interval, last, frame_index = .05, 0.0, 0
        self._open_log()
        self._set_status(running=True, state="ready", detail="仅白线跟随与红终点停车；按 M 开始")
        try:
            while not self._stop.is_set():
                now, jpeg = time.monotonic(), self.camera.latest_jpeg()
                if jpeg is None or now - last < interval:
                    self._stop.wait(.003)
                    continue
                last = now
                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                result = find_fast_line(image, self._last_center_x, FastLineConfig())
                if result.center_x is not None:
                    self._last_center_x = result.center_x
                red = self._red_detector.detect(image)
                decision = self._planner.step(line_valid=result.valid, red_band=red, frame_height=image.shape[0])
                state, motor, commanded = decision.state.value, "PAUSED", None
                if self.gate.enabled():
                    if decision.stop:
                        self._stop_motor()
                        motor = "STOP_RED_TERMINAL" if red.detected else "STOP_LINE_LOST"
                    else:
                        commanded = pwm_for_line(result, image.shape[1], 85, FastLineConfig())
                        if commanded is None:
                            self._stop_motor()
                            state, motor = "STOPPED_UNSAFE_LINE_LOST", "STOP_NO_NEAR_WHITE_LINE"
                        else:
                            self.controller.set_direct_drive(*commanded)
                            self._motor_active = True
                            motor = f"FOLLOW_PWM R={commanded[0]} L={commanded[1]}"
                else:
                    self._stop_motor()
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
