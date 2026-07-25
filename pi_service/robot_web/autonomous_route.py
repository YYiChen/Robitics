"""In-process route preview and M-key controlled autonomous driving."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
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
    process_fps: float = 20.0
    straight_pwm: int = 95
    launch_pwm: int = 155
    lookahead_gain: float = 200.0
    heading_weight: float = 0.25
    correction_deadband: float = 0.01
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 130
    minimum_wheel_pwm: int = 0
    maximum_wheel_pwm: int = 200
    sharp_turn_error: float = 0.08
    sharp_turn_correction_pwm: int = 55
    sharp_turn_inner_pwm: int = 0


class AutonomousRouteTracker:
    """Run the existing green/white detector from the same process as port 5000."""
    def __init__(self, controller, camera, publisher: RoutePreviewPublisher, gate: AutonomousRunGate, config: AutonomousRouteConfig) -> None:
        self.controller, self.camera, self.publisher, self.gate, self.config = controller, camera, publisher, gate, config
        self._stop = threading.Event(); self._thread: threading.Thread | None = None; self._lock = threading.Lock()
        self._motor_active = False; self._launch_until = 0.0
        self._status = {"available": True, "running": False, "enabled": False, "state": "starting", "detail": "等待路线识别器启动", "frame": 0}

    def start(self) -> None:
        if self._thread and self._thread.is_alive(): return
        self._stop.clear(); self._thread = threading.Thread(target=self._run, daemon=True, name="autonomous-route"); self._thread.start()

    def stop(self) -> None:
        self._stop.set(); self._stop_motor()
        if self._thread and self._thread is not threading.current_thread(): self._thread.join(timeout=1.0)

    def status_dict(self) -> dict:
        with self._lock: status = dict(self._status)
        status["enabled"] = self.gate.enabled()
        return status

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled: self._stop_motor()
        self._set_status(enabled=enabled, detail="自动行驶已开启" if enabled else "自动行驶已暂停，电机已停止")
        return self.status_dict()

    def _set_status(self, **changes) -> None:
        with self._lock: self._status.update(changes)

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now(); self._motor_active = False; self._launch_until = 0.0

    def _bounded(self, value: int) -> int:
        return max(-self.config.maximum_wheel_pwm, min(self.config.maximum_wheel_pwm, value))

    def _draw(self, cv2, frame, result, decision, marker, motor_text: str):
        from track_line.visualization import render_debug
        output = render_debug(frame, result)
        enabled = self.gate.enabled(); color = (0, 220, 0) if enabled else (0, 180, 255)
        cv2.rectangle(output, (10, 76), (800, 222), (20, 20, 20), cv2.FILLED)
        cv2.putText(output, f"AUTONOMOUS: {'RUNNING' if enabled else 'PAUSED (press M)'}", (18, 104), cv2.FONT_HERSHEY_SIMPLEX, .68, color, 2, cv2.LINE_AA)
        cv2.putText(output, f"PATH: {decision.intent.value}  {decision.reason}", (18, 131), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)
        lookahead = "n/a" if result.observation.lookahead_offset is None else f"{result.observation.lookahead_offset:+.3f}"
        cv2.putText(output, f"LOOKAHEAD: {lookahead}  MOTOR: {motor_text}", (18, 158), cv2.FONT_HERSHEY_SIMPLEX, .48, (100, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(output, f"MARKER: {marker.marker_in_lap}/4 lap={marker.lap_count}", (18, 184), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(output, "M: start/pause automatic driving; vision keeps running", (18, 210), cv2.FONT_HERSHEY_SIMPLEX, .42, (190, 190, 190), 1, cv2.LINE_AA)
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
            planner = ContinuousPathPlanner(ContinuousPathConfig())
            marker_counter = MarkerCounter(MarkerCounterConfig(confirm_frames=3, clear_frames=4, markers_per_lap=4, rearm_y_drop_ratio=.55))
            motor = ContinuousMotorConfig("in-process", straight_pwm=self.config.straight_pwm, launch_pwm=self.config.launch_pwm, lookahead_gain=self.config.lookahead_gain, heading_weight=self.config.heading_weight, correction_deadband=self.config.correction_deadband, minimum_correction_pwm=self.config.minimum_correction_pwm, maximum_correction_pwm=self.config.maximum_correction_pwm, minimum_wheel_pwm=self.config.minimum_wheel_pwm, sharp_turn_error=self.config.sharp_turn_error, sharp_turn_correction_pwm=self.config.sharp_turn_correction_pwm, sharp_turn_inner_pwm=self.config.sharp_turn_inner_pwm)
            self._set_status(running=True, state="ready", detail="视觉识别运行中；按 M 开启自动行驶")
            last, index, last_pair, interval = 0.0, 0, None, 1.0 / max(1.0, self.config.process_fps)
            while not self._stop.is_set():
                jpeg, now = self.camera.latest_jpeg(), time.monotonic()
                if jpeg is None or now - last < interval:
                    self._stop.wait(.005); continue
                last = now; frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue
                result = detector.detect(frame, frame_index=index, timestamp_ns=time.monotonic_ns())
                marker = marker_counter.update(result.observation.marker_detected, result.observation.marker_y_ratio)
                decision = planner.step(result.observation, now); motor_text = "PAUSED"
                if self.gate.enabled() and decision.intent is PathIntent.FOLLOW_PATH:
                    if not self._motor_active:
                        self._motor_active = True; self._launch_until = now + motor.launch_duration_seconds
                    if now < self._launch_until:
                        pair, motor_text = (self._bounded(motor.launch_pwm), self._bounded(motor.launch_pwm)), "LAUNCH"
                    elif result.observation.lookahead_offset is None and last_pair is not None: pair, motor_text = last_pair, "HOLD_LAST_PATH"
                    else:
                        pair, error, correction = path_drive_details(result.observation, motor); pair = tuple(self._bounded(p) for p in pair); last_pair = pair
                        motor_text = f"P e={error:+.3f} c={correction} R={pair[0]} L={pair[1]}" if error is not None else f"R={pair[0]} L={pair[1]}"
                    self.controller.set_direct_drive(*pair)
                else:
                    if decision.intent is PathIntent.STOP: last_pair = None
                    self._stop_motor()
                annotated = self._draw(cv2, frame, result, decision, marker, motor_text)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok: self.publisher.publish(encoded.tobytes())
                self._set_status(state=decision.intent.value, detail=decision.reason, frame=index, marker_in_lap=marker.marker_in_lap, lap_count=marker.lap_count, confidence=result.observation.confidence)
                index += 1
        except Exception as exc:
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor(); self._set_status(running=False)
