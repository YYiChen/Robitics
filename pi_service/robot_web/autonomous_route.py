"""In-process route preview and M-key controlled autonomous driving."""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from pathlib import Path
import re
import sys
import threading
import time
from typing import Iterator


class AutonomousRunGate:
    """Motor permission; deliberately starts paused so service startup is safe."""
    def __init__(self) -> None:
        self._enabled = False
        self._lock = threading.Lock()

    def toggle(self) -> bool:
        with self._lock:
            self._enabled = not self._enabled
            return self._enabled

    def enabled(self) -> bool:
        with self._lock:
            return self._enabled


class RoutePreviewPublisher:
    """A latest-frame MJPEG publisher; slow browser clients never build a queue."""
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0

    def publish(self, jpeg: bytes) -> None:
        with self._condition:
            self._jpeg, self._sequence = jpeg, self._sequence + 1
            self._condition.notify_all()

    def iter_mjpeg(self) -> Iterator[bytes]:
        sequence = -1
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._sequence != sequence, timeout=1.0)
                jpeg, sequence = self._jpeg, self._sequence
            if jpeg:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"


@dataclass(frozen=True)
class AutonomousRouteConfig:
    detector_config: Path
    tuning_path: Path | None = None
    process_fps: float = 20.0
    straight_pwm: int = 95
    launch_pwm: int = 155
    lookahead_gain: float = 200.0
    heading_weight: float = 0.25
    correction_deadband: float = 0.01
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 130
    minimum_wheel_pwm: int = 55
    maximum_wheel_pwm: int = 200
    sharp_turn_error: float = 0.08
    sharp_turn_correction_pwm: int = 55
    sharp_turn_inner_pwm: int = -200
    sharp_turn_outer_pwm: int = 200
    line_lost_stop_frames: int = 3
    line_lost_prediction_seconds: float = 0.75
    line_lost_stop_seconds: float = 1.0
    marker_confirm_frames: int = 2
    marker_clear_frames: int = 12
    marker_rearm_y_drop_ratio: float = 0.18
    markers_per_lap: int = 4
    motion_observe_seconds: float = 1.0
    motion_min_path_shift_px: float = 25.0
    motion_step_pwm: int = 5


TUNING_FIELDS = {
    "process_fps": ("PROCESS_FPS", float, 1.0, 120.0),
    "line_lost_stop_frames": ("LINE_LOST_STOP_FRAMES", int, 1, 1000),
    "line_lost_prediction_seconds": ("LINE_LOST_PREDICTION_SECONDS", float, 0.0, 60.0),
    "line_lost_stop_seconds": ("LINE_LOST_STOP_SECONDS", float, 0.0, 60.0),
    "marker_confirm_frames": ("MARKER_CONFIRM_FRAMES", int, 1, 1000),
    "marker_clear_frames": ("MARKER_CLEAR_FRAMES", int, 1, 1000),
    "marker_rearm_y_drop_ratio": ("MARKER_REARM_Y_DROP_RATIO", float, 0.0, 1.0),
    "markers_per_lap": ("MARKERS_PER_LAP", int, 1, 1000),
    "straight_pwm": ("STRAIGHT_PWM", int, 0, 255),
    "launch_pwm": ("LAUNCH_PWM", int, 0, 255),
    "lookahead_gain": ("LOOKAHEAD_GAIN", float, 0.0, 10000.0),
    "heading_weight": ("HEADING_WEIGHT", float, 0.0, 10.0),
    "correction_deadband": ("CORRECTION_DEADBAND", float, 0.0, 1.0),
    "minimum_correction_pwm": ("MINIMUM_CORRECTION_PWM", int, 0, 255),
    "maximum_correction_pwm": ("MAXIMUM_CORRECTION_PWM", int, 0, 255),
    "minimum_wheel_pwm": ("MINIMUM_WHEEL_PWM", int, 0, 255),
    "maximum_wheel_pwm": ("MAXIMUM_WHEEL_PWM", int, 0, 255),
    "sharp_turn_error": ("SHARP_TURN_ERROR", float, 0.0, 1.0),
    "sharp_turn_correction_pwm": ("SHARP_TURN_CORRECTION_PWM", int, 0, 255),
    "sharp_turn_inner_pwm": ("SHARP_TURN_INNER_PWM", int, -255, 255),
    "sharp_turn_outer_pwm": ("SHARP_TURN_OUTER_PWM", int, -255, 255),
    "motion_observe_seconds": ("MOTION_OBSERVE_SECONDS", float, 0.1, 60.0),
    "motion_min_path_shift_px": ("MOTION_MIN_PATH_SHIFT_PX", float, 1.0, 1000.0),
    "motion_step_pwm": ("MOTION_STEP_PWM", int, 1, 255),
}


def load_tuning_config(detector_config: Path, tuning_path: Path) -> AutonomousRouteConfig:
    """Load trusted constant assignments from the existing tuning.py file."""
    values: dict[str, object] = {}
    namespace: dict[str, object] = {}
    exec(tuning_path.read_text(encoding="utf-8"), namespace)
    for field, (constant, converter, _low, _high) in TUNING_FIELDS.items():
        if constant in namespace:
            values[field] = converter(namespace[constant])
    return AutonomousRouteConfig(detector_config=detector_config, tuning_path=tuning_path, **values)


class AutonomousRouteTracker:
    """Run the existing green/white detector from the same process as port 5000."""
    def __init__(self, controller, camera, publisher: RoutePreviewPublisher, gate: AutonomousRunGate, config: AutonomousRouteConfig) -> None:
        self.controller, self.camera, self.publisher, self.gate, self.config = controller, camera, publisher, gate, config
        self._stop = threading.Event(); self._thread: threading.Thread | None = None; self._lock = threading.Lock()
        self._motor_active = False; self._launch_until = 0.0; self._tuning_lock = threading.RLock(); self._tuning_version = 0
        self._motion_reference: tuple[float, tuple[tuple[float, float], ...]] | None = None
        self._motion_boost_pwm: int | None = None; self._motion_confirmed = False; self._motion_text = "WAIT_STRAIGHT"
        self._startup_reacquire_frames = 0
        self._status = {"available": True, "running": False, "enabled": False, "state": "starting", "detail": "等待路线识别器启动", "frame": 0}

    def start(self) -> None:
        if self._thread and self._thread.is_alive(): return
        self._stop.clear(); self._thread = threading.Thread(target=self._run, daemon=True, name="autonomous-route"); self._thread.start()

    def stop(self) -> None:
        self._stop.set(); self._stop_motor()
        if self._thread and self._thread is not threading.current_thread(): self._thread.join(timeout=1.0)

    def status_dict(self) -> dict:
        with self._lock: status = dict(self._status)
        with self._tuning_lock:
            status["tuning"] = {key: getattr(self.config, key) for key in TUNING_FIELDS}
        status["enabled"] = self.gate.enabled()
        return status

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled: self._stop_motor()
        self._set_status(enabled=enabled, detail="自动行驶已开启" if enabled else "自动行驶已暂停，电机已停止")
        return self.status_dict()

    def update_tuning(self, payload: dict) -> dict:
        """Validate, persist and atomically publish a new route-control setup."""
        changes: dict[str, object] = {}
        for field, (_constant, converter, low, high) in TUNING_FIELDS.items():
            if field not in payload:
                continue
            try:
                value = converter(payload[field])
            except (TypeError, ValueError):
                raise ValueError(f"{field} 必须是有效数字") from None
            if not low <= value <= high:
                raise ValueError(f"{field} 必须在 {low} 到 {high} 之间")
            changes[field] = value
        if not changes:
            raise ValueError("没有可更新的循迹参数")
        if changes.get("line_lost_prediction_seconds", self.config.line_lost_prediction_seconds) > changes.get("line_lost_stop_seconds", self.config.line_lost_stop_seconds):
            raise ValueError("盲区预测时间不能大于最终停车时间")
        if changes.get("minimum_correction_pwm", self.config.minimum_correction_pwm) > changes.get("maximum_correction_pwm", self.config.maximum_correction_pwm):
            raise ValueError("最小转向差不能大于最大转向差")
        if changes.get("minimum_wheel_pwm", self.config.minimum_wheel_pwm) > changes.get("maximum_wheel_pwm", self.config.maximum_wheel_pwm):
            raise ValueError("最小轮速不能大于最大轮速")
        with self._tuning_lock:
            updated = replace(self.config, **changes)
            if updated.tuning_path:
                source = updated.tuning_path.read_text(encoding="utf-8")
                for field, value in changes.items():
                    constant = TUNING_FIELDS[field][0]
                    source, count = re.subn(rf"(?m)^{constant}\\s*=\\s*[^\\n#]+", f"{constant} = {value!r}", source, count=1)
                    if count != 1:
                        source += f"\n{constant} = {value!r}\n"
                updated.tuning_path.write_text(source, encoding="utf-8")
            self.config = updated
            self._tuning_version += 1
        self._set_status(detail="循迹参数已实时应用并保存到 tuning.py")
        return self.status_dict()

    def _set_status(self, **changes) -> None:
        with self._lock: self._status.update(changes)

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now(); self._motor_active = False; self._launch_until = 0.0

    def _reset_motion_assist(self) -> None:
        self._motion_reference = None; self._motion_boost_pwm = None; self._motion_confirmed = False; self._motion_text = "WAIT_STRAIGHT"

    @staticmethod
    def _path_signature(path: tuple[tuple[int, int], ...], samples: int = 12) -> tuple[tuple[float, float], ...] | None:
        if len(path) < 2:
            return None
        return tuple(tuple(map(float, path[round(i * (len(path) - 1) / (samples - 1))])) for i in range(samples))

    @staticmethod
    def _signature_shift(first: tuple[tuple[float, float], ...], second: tuple[tuple[float, float], ...]) -> float:
        return sum(((ax - bx) ** 2 + (ay - by) ** 2) ** .5 for (ax, ay), (bx, by) in zip(first, second)) / max(1, len(first))

    def _motion_assist_pwm(self, now: float, path: tuple[tuple[int, int], ...], straight: bool, config: AutonomousRouteConfig) -> tuple[int, str]:
        """Increase only a stationary straight-line launch, based on route motion."""
        if not straight:
            self._reset_motion_assist()
            return config.straight_pwm, self._motion_text
        signature = self._path_signature(path)
        if signature is None:
            self._motion_text = "NO_ROUTE_SIGNATURE"
            return config.straight_pwm, self._motion_text
        if self._motion_confirmed:
            self._motion_text = "MOVING_CONFIRMED"
            return config.straight_pwm, self._motion_text
        if self._motion_reference is None:
            self._motion_reference = (now, signature); self._motion_text = "OBSERVING_ROUTE"
            return self._motion_boost_pwm or config.straight_pwm, self._motion_text
        started, reference = self._motion_reference
        if now - started < config.motion_observe_seconds:
            self._motion_text = f"OBSERVING {now - started:.1f}/{config.motion_observe_seconds:.1f}s"
            return self._motion_boost_pwm or config.straight_pwm, self._motion_text
        shift = self._signature_shift(reference, signature)
        self._motion_reference = (now, signature)
        if shift >= config.motion_min_path_shift_px:
            self._motion_confirmed = True; self._motion_boost_pwm = None
            self._motion_text = f"MOVING shift={shift:.1f}px"
            return config.straight_pwm, self._motion_text
        current = self._motion_boost_pwm if self._motion_boost_pwm is not None else config.straight_pwm
        boosted = min(config.maximum_wheel_pwm, current + config.motion_step_pwm)
        self._motion_boost_pwm = boosted
        self._motion_text = f"STILL shift={shift:.1f}px -> PWM {boosted}"
        return boosted, self._motion_text

    def _bounded(self, value: int) -> int:
        return max(-self.config.maximum_wheel_pwm, min(self.config.maximum_wheel_pwm, value))

    def _draw(self, cv2, frame, result, decision, marker, motor_text: str):
        from track_line.visualization import render_debug
        output = render_debug(frame, result)
        enabled = self.gate.enabled(); color = (0, 220, 0) if enabled else (0, 180, 255)
        cv2.rectangle(output, (10, 76), (800, 246), (20, 20, 20), cv2.FILLED)
        cv2.putText(output, f"AUTONOMOUS: {'RUNNING' if enabled else 'PAUSED (press M)'}", (18, 104), cv2.FONT_HERSHEY_SIMPLEX, .68, color, 2, cv2.LINE_AA)
        cv2.putText(output, f"PATH: {decision.intent.value}  {decision.reason}", (18, 131), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)
        lookahead = "n/a" if result.observation.lookahead_offset is None else f"{result.observation.lookahead_offset:+.3f}"
        cv2.putText(output, f"LOOKAHEAD: {lookahead}  MOTOR: {motor_text}", (18, 158), cv2.FONT_HERSHEY_SIMPLEX, .48, (100, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(output, f"MARKER: {marker.marker_in_lap}/4 lap={marker.lap_count}", (18, 184), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(output, f"START ASSIST: {self._motion_text}", (18, 210), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 210, 80), 1, cv2.LINE_AA)
        cv2.putText(output, "M: start/pause automatic driving; vision keeps running", (18, 234), cv2.FONT_HERSHEY_SIMPLEX, .42, (190, 190, 190), 1, cv2.LINE_AA)
        return output

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np
            root = Path(__file__).resolve().parents[2]
            track_src = root / "third_party" / "DeskMate-Advance" / "src"
            experiment = root / "pi_service" / "experiments" / "continuous_path_validation"
            if not track_src.is_dir() or not self.config.detector_config.is_file(): raise RuntimeError(f"路线配置不存在: {self.config.detector_config}")
            sys.path[:0] = [str(track_src), str(root), str(experiment)]
            from track_line.config import LineDetectorConfig
            from track_line.detector import OpenCVLineDetector
            from continuous_path_planner import ContinuousPathConfig, ContinuousPathPlanner, PathIntent
            from continuous_motor_control import ContinuousMotorConfig, path_drive_details
            from marker_counter import MarkerCounter, MarkerCounterConfig
            detector = OpenCVLineDetector(LineDetectorConfig.from_json(self.config.detector_config))
            self._set_status(running=True, state="ready", detail="视觉识别运行中；按 M 开启自动行驶")
            last, index, last_pair, interval, applied_version = 0.0, 0, None, 0.05, -1
            while not self._stop.is_set():
                with self._tuning_lock:
                    config, version = self.config, self._tuning_version
                if version != applied_version:
                    planner = ContinuousPathPlanner(ContinuousPathConfig(minimum_confidence=.38, line_lost_stop_frames=config.line_lost_stop_frames, line_lost_prediction_seconds=config.line_lost_prediction_seconds, line_lost_stop_seconds=config.line_lost_stop_seconds))
                    marker_counter = MarkerCounter(MarkerCounterConfig(confirm_frames=config.marker_confirm_frames, clear_frames=config.marker_clear_frames, markers_per_lap=config.markers_per_lap, rearm_y_drop_ratio=config.marker_rearm_y_drop_ratio))
                    motor = ContinuousMotorConfig("in-process", straight_pwm=config.straight_pwm, launch_pwm=config.launch_pwm, lookahead_gain=config.lookahead_gain, heading_weight=config.heading_weight, correction_deadband=config.correction_deadband, minimum_correction_pwm=config.minimum_correction_pwm, maximum_correction_pwm=config.maximum_correction_pwm, minimum_wheel_pwm=config.minimum_wheel_pwm, sharp_turn_error=config.sharp_turn_error, sharp_turn_correction_pwm=config.sharp_turn_correction_pwm, sharp_turn_inner_pwm=config.sharp_turn_inner_pwm, sharp_turn_outer_pwm=config.sharp_turn_outer_pwm)
                    interval, applied_version = 1.0 / max(1.0, config.process_fps), version
                jpeg, now = self.camera.latest_jpeg(), time.monotonic()
                if jpeg is None or now - last < interval:
                    self._stop.wait(.005); continue
                last = now; frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue
                result = detector.detect(frame, frame_index=index, timestamp_ns=time.monotonic_ns())
                marker = marker_counter.update(result.observation.marker_detected, result.observation.marker_y_ratio)
                decision = planner.step(result.observation, now); motor_text = "PAUSED"
                # The web service intentionally begins with motors paused. It
                # may therefore see no tape during camera/service startup.
                # Once M has armed driving, three consecutive valid route
                # frames are enough to safely leave that startup-only lock.
                visible = (
                    not result.observation.line_lost
                    and result.observation.confidence >= .38
                    and result.observation.lookahead_offset is not None
                )
                if decision.reason == "startup_line_missing":
                    self._startup_reacquire_frames = self._startup_reacquire_frames + 1 if visible else 0
                    if self.gate.enabled() and self._startup_reacquire_frames >= 3:
                        planner = ContinuousPathPlanner(ContinuousPathConfig(minimum_confidence=.38, line_lost_stop_frames=config.line_lost_stop_frames, line_lost_prediction_seconds=config.line_lost_prediction_seconds, line_lost_stop_seconds=config.line_lost_stop_seconds))
                        decision = planner.step(result.observation, now)
                        self._startup_reacquire_frames = 0
                        self._set_status(detail="路线稳定重获，已解除启动丢线锁")
                else:
                    self._startup_reacquire_frames = 0
                if self.gate.enabled() and decision.intent is PathIntent.FOLLOW_PATH:
                    if not self._motor_active:
                        self._motor_active = True; self._launch_until = now + motor.launch_duration_seconds
                    if now < self._launch_until:
                        pair, motor_text = (self._bounded(motor.launch_pwm), self._bounded(motor.launch_pwm)), "LAUNCH"
                    elif result.observation.lookahead_offset is None and last_pair is not None: pair, motor_text = last_pair, "HOLD_LAST_PATH"; self._reset_motion_assist()
                    else:
                        pair, error, correction = path_drive_details(result.observation, motor)
                        straight = error is not None and abs(error) <= config.correction_deadband
                        assisted_pwm, assist_text = self._motion_assist_pwm(now, result.centerline_px, straight, config)
                        if assisted_pwm != config.straight_pwm:
                            assisted_motor = replace(motor, straight_pwm=assisted_pwm)
                            pair, error, correction = path_drive_details(result.observation, assisted_motor)
                        pair = tuple(self._bounded(p) for p in pair); last_pair = pair
                        motor_text = f"P e={error:+.3f} c={correction} R={pair[0]} L={pair[1]}" if error is not None else f"R={pair[0]} L={pair[1]}"
                        motor_text += f" {assist_text}"
                    self.controller.set_direct_drive(*pair)
                else:
                    if decision.intent is PathIntent.STOP: last_pair = None
                    self._reset_motion_assist()
                    self._stop_motor()
                annotated = self._draw(cv2, frame, result, decision, marker, motor_text)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok: self.publisher.publish(encoded.tobytes())
                self._set_status(state=decision.intent.value, detail=decision.reason, frame=index, marker_in_lap=marker.marker_in_lap, lap_count=marker.lap_count, confidence=result.observation.confidence, motion_assist=self._motion_text, motion_boost_pwm=self._motion_boost_pwm)
                index += 1
        except Exception as exc:
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor(); self._set_status(running=False)
