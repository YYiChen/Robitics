"""Port-5000 adapter for the isolated scanline I-shape turnaround algorithm."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[2]
SCANLINE_EXPERIMENT = ROOT / "pi_service" / "experiments" / "i_shape_scanline_turnaround_validation"
if str(SCANLINE_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(SCANLINE_EXPERIMENT))

from scanline_i_logic import IShapeScanlineAnalyzer, IShapeTurnaroundPlanner, TurnaroundConfig, TurnaroundState  # noqa: E402


@dataclass(frozen=True)
class ScanlineIRouteConfig:
    process_fps: float = 20.0
    straight_pwm: int = 120
    pivot_pwm: int = 200
    pivot_min_seconds: float = 2.5
    pivot_max_seconds: float = 5.0


class ScanlineIShapeRouteTracker:
    """Keep the main console's camera, M gate, and preview while reusing the isolated detector."""
    def __init__(self, controller, camera, publisher, gate, config: ScanlineIRouteConfig = ScanlineIRouteConfig()) -> None:
        self.controller, self.camera, self.publisher, self.gate, self.config = controller, camera, publisher, gate, config
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._motor_active = False
        self._status = {"available": True, "running": False, "enabled": False, "mode": "scanline_i", "state": "starting", "detail": "正在启动扫描线 I 型识别", "frame": 0, "confidence": None}

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="scanline-i-route")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._stop_motor()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _set_status(self, **changes: object) -> None:
        with self._lock:
            self._status.update(changes)

    def status_dict(self) -> dict:
        with self._lock:
            status = dict(self._status)
        status["enabled"] = self.gate.enabled()
        status["tuning"] = asdict(self.config)
        return status

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled:
            self._stop_motor()
        self._set_status(enabled=enabled, detail="扫描线自动行驶已开启" if enabled else "扫描线自动行驶已暂停，电机已停止")
        return self.status_dict()

    def update_tuning(self, _payload: dict) -> dict:
        raise ValueError("扫描线 I 型模式使用固定安全参数；请先在隔离实验验证后再调整")

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now()
            self._motor_active = False

    def _draw(self, cv2, frame, result, decision, motor_text: str):
        output = frame.copy()
        mask = cv2.cvtColor(result.component_mask, cv2.COLOR_GRAY2BGR)
        output = cv2.addWeighted(output, .72, mask, .28, 0)
        evidence = result.evidence
        for y, x, _width in evidence.line_centers:
            cv2.circle(output, (int(x), y), 5, (0, 255, 0), -1)
        if evidence.endpoint_y is not None:
            cv2.line(output, (0, evidence.endpoint_y), (output.shape[1] - 1, evidence.endpoint_y), (0, 165, 255), 2)
        color = (0, 220, 0) if self.gate.enabled() else (0, 180, 255)
        cv2.rectangle(output, (10, 76), (940, 220), (20, 20, 20), cv2.FILLED)
        cv2.putText(output, f"SCANLINE I-TURN: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18, 104), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
        cv2.putText(output, f"STATE: {decision.state.value}  {decision.reason}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
        cv2.putText(output, f"BAR: {evidence.endpoint_detected} y={evidence.endpoint_y} width={evidence.endpoint_width}  MOTOR: {motor_text}", (18, 160), cv2.FONT_HERSHEY_SIMPLEX, .46, (100, 220, 255), 1)
        cv2.putText(output, f"CONF: {evidence.confidence:.2f}  narrow-centre={evidence.line_center_x}  M: start/pause", (18, 188), cv2.FONT_HERSHEY_SIMPLEX, .44, (190, 190, 190), 1)
        cv2.putText(output, "Endpoint bar is never followed as a left/right path.", (18, 212), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 220, 255), 1)
        return output

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np

            analyzer = IShapeScanlineAnalyzer()
            planner = IShapeTurnaroundPlanner(TurnaroundConfig(pivot_min_seconds=self.config.pivot_min_seconds, pivot_max_seconds=self.config.pivot_max_seconds))
            self._set_status(running=True, state="ready", detail="扫描线 I 型识别运行中；按 M 开启自动行驶")
            interval, last, frame_index = 1.0 / max(1.0, self.config.process_fps), 0.0, 0
            while not self._stop.is_set():
                jpeg, now = self.camera.latest_jpeg(), time.monotonic()
                if jpeg is None or now - last < interval:
                    self._stop.wait(.005)
                    continue
                last = now
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                result = analyzer.analyze(frame)
                evidence = result.evidence
                decision = planner.step(evidence, now)
                motor_text = "PAUSED"
                if self.gate.enabled():
                    if decision.state is TurnaroundState.FOLLOW_STRAIGHT and evidence.valid_line and not evidence.endpoint_detected:
                        self.controller.set_direct_drive(self.config.straight_pwm, self.config.straight_pwm)
                        self._motor_active, motor_text = True, f"STRAIGHT R={self.config.straight_pwm} L={self.config.straight_pwm}"
                    elif decision.state is TurnaroundState.PIVOT_180:
                        # Keep the same right-pivot sign convention as the main route mode.
                        self.controller.set_direct_drive(-self.config.pivot_pwm, self.config.pivot_pwm)
                        self._motor_active, motor_text = True, f"PIVOT_RIGHT R={-self.config.pivot_pwm} L={self.config.pivot_pwm}"
                    else:
                        self._stop_motor()
                        motor_text = "STOP_WAITING_FOR_ENDPOINT_CONFIRMATION"
                else:
                    self._stop_motor()
                annotated = self._draw(cv2, frame, result, decision, motor_text)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                self._set_status(state=decision.state.value, detail=decision.reason, frame=frame_index, confidence=evidence.confidence, endpoint_detected=evidence.endpoint_detected, endpoint_y=evidence.endpoint_y, endpoint_width=evidence.endpoint_width, motor=motor_text)
                frame_index += 1
        except Exception as exc:
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            self._set_status(running=False)
