"""Port-5000 adapter for the isolated scanline I-shape turnaround algorithm."""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
SCANLINE_EXPERIMENT = ROOT / "pi_service" / "experiments" / "i_shape_scanline_turnaround_validation"
STRAIGHT_LINE_EXPERIMENT = ROOT / "pi_service" / "experiments" / "straight_line_stop_validation"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(SCANLINE_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(SCANLINE_EXPERIMENT))
if str(STRAIGHT_LINE_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(STRAIGHT_LINE_EXPERIMENT))

from scanline_i_logic import (  # noqa: E402
    HybridScanlineAnalyzer,
    HybridScanlineConfig,
    IShapeScanlineAnalyzer,
    IShapeTurnaroundPlanner,
    TurnaroundConfig,
    TurnaroundState,
)
from straight_motor_control import StraightMotorConfig, drive_pwm_for_offset  # noqa: E402


FORWARD_TRACKING_STATES = frozenset((
    TurnaroundState.FOLLOW_STRAIGHT,
    TurnaroundState.EARLY_BAR_PREDICTED,
    TurnaroundState.BAR_MARKED,
))


@dataclass(frozen=True)
class ScanlineIRouteConfig:
    process_fps: float = 20.0
    straight_pwm: int = 120
    pivot_pwm: int = 200
    correction_deadband: float = 0.05
    correction_gain: float = 120.0
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 60
    pivot_min_seconds: float = 2.5
    pivot_max_seconds: float = 5.0
    tuning_path: Path | None = None
    use_hybrid: bool = True


SCANLINE_TUNING_FIELDS = {
    "straight_pwm": (int, 0, 255),
    "pivot_pwm": (int, 0, 255),
    "correction_deadband": (float, 0.0, 1.0),
    "correction_gain": (float, 0.0, 1000.0),
    "minimum_correction_pwm": (int, 0, 255),
    "maximum_correction_pwm": (int, 0, 255),
    "pivot_min_seconds": (float, 0.0, 20.0),
    "pivot_max_seconds": (float, 0.1, 30.0),
}


def load_scanline_tuning_config(tuning_path: Path) -> ScanlineIRouteConfig:
    """Load only the small, I-shape-specific web tuning file when it exists."""
    values: dict[str, object] = {}
    if tuning_path.exists():
        try:
            stored = json.loads(tuning_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            stored = {}
        if isinstance(stored, dict):
            for field, (converter, low, high) in SCANLINE_TUNING_FIELDS.items():
                try:
                    value = converter(stored[field])
                except (KeyError, TypeError, ValueError):
                    continue
                if low <= value <= high:
                    values[field] = value
    return ScanlineIRouteConfig(tuning_path=tuning_path, **values)


class ScanlineIShapeRouteTracker:
    """Keep the main console's camera, M gate, and preview while reusing the isolated detector."""
    def __init__(self, controller, camera, publisher, gate, config: ScanlineIRouteConfig = ScanlineIRouteConfig()) -> None:
        self.controller, self.camera, self.publisher, self.gate, self.config = controller, camera, publisher, gate, config
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._tuning_lock = threading.RLock()
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
        with self._tuning_lock:
            status["tuning"] = {field: getattr(self.config, field) for field in SCANLINE_TUNING_FIELDS}
        return status

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled:
            self._stop_motor()
        self._set_status(enabled=enabled, detail="扫描线自动行驶已开启" if enabled else "扫描线自动行驶已暂停，电机已停止")
        return self.status_dict()

    def update_tuning(self, payload: dict) -> dict:
        """Apply and persist the speeds used only by the scanline I experiment."""
        changes: dict[str, object] = {}
        for field, (converter, low, high) in SCANLINE_TUNING_FIELDS.items():
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
            raise ValueError("没有可更新的扫描线 I 型参数")
        with self._tuning_lock:
            updated = replace(self.config, **changes)
            if updated.minimum_correction_pwm > updated.maximum_correction_pwm:
                raise ValueError("最小直线修正 PWM 不能大于最大直线修正 PWM")
            if updated.pivot_min_seconds > updated.pivot_max_seconds:
                raise ValueError("最短掉头时间不能大于最长掉头保险时间")
            if updated.tuning_path:
                updated.tuning_path.parent.mkdir(parents=True, exist_ok=True)
                values = {field: getattr(updated, field) for field in SCANLINE_TUNING_FIELDS}
                updated.tuning_path.write_text(json.dumps(values, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            self.config = updated
        self._set_status(detail="扫描线 I 型直行与掉头参数已实时应用并保存")
        return self.status_dict()

    @staticmethod
    def _straight_pair(evidence, frame_width: int, config: ScanlineIRouteConfig) -> tuple[int, int]:
        offset = None if evidence.line_center_x is None else (evidence.line_center_x - frame_width / 2.0) / max(1.0, frame_width / 2.0)
        observation = SimpleNamespace(offset=offset, valid_bands=len(evidence.line_centers))
        motor = StraightMotorConfig(
            controller_url="in-process",
            straight_pwm=config.straight_pwm,
            correction_deadband=config.correction_deadband,
            correction_gain=config.correction_gain,
            minimum_correction_pwm=config.minimum_correction_pwm,
            maximum_correction_pwm=config.maximum_correction_pwm,
        )
        return drive_pwm_for_offset(observation, motor)

    @staticmethod
    def _keeps_forward_motion(state: TurnaroundState) -> bool:
        """A far junction is a prediction only; it must never brake the car."""
        return state in FORWARD_TRACKING_STATES

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now()
            self._motor_active = False

    def _draw(self, cv2, frame, result, decision, motor_text: str):
        # Yellow, not grayscale: this is the selected near-anchored route
        # component used by control, so it remains legible on a grey floor.
        yellow_route = frame.copy()
        yellow_route[result.component_mask > 0] = (0, 220, 255)
        output = cv2.addWeighted(frame, .62, yellow_route, .38, 0)
        evidence = result.evidence
        for y, x, _width in evidence.line_centers:
            cv2.circle(output, (int(x), y), 5, (0, 255, 0), -1)
        if evidence.endpoint_y is not None:
            cv2.line(output, (0, evidence.endpoint_y), (output.shape[1] - 1, evidence.endpoint_y), (0, 165, 255), 2)
        # Hybrid: draw lookahead point (yellow) and junction line (magenta)
        if evidence.lookahead_x is not None and evidence.lookahead_y is not None:
            cv2.circle(output, (int(evidence.lookahead_x), evidence.lookahead_y), 7, (0, 255, 255), -1)
        if evidence.junction_detected and evidence.junction_y is not None:
            cv2.line(output, (0, evidence.junction_y), (output.shape[1] - 1, evidence.junction_y), (255, 0, 255), 1)
        color = (0, 220, 0) if self.gate.enabled() else (0, 180, 255)
        bar_y = max(76 + 24 * 6, 220)
        cv2.rectangle(output, (10, 76), (940, 250), (20, 20, 20), cv2.FILLED)
        cv2.putText(output, f"SCANLINE I-TURN: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18, 104), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
        cv2.putText(output, f"STATE: {decision.state.value}  {decision.reason}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
        cv2.putText(output, f"BAR: {evidence.endpoint_detected} y={evidence.endpoint_y} w={evidence.endpoint_width}  JUNCTION: {evidence.junction_detected} y={evidence.junction_y} arms={evidence.junction_arm_count}", (18, 160), cv2.FONT_HERSHEY_SIMPLEX, .46, (100, 220, 255), 1)
        cv2.putText(output, f"LOOKAHEAD: ({evidence.lookahead_x}, {evidence.lookahead_y}) path={evidence.path_length_px}px  MOTOR: {motor_text}", (18, 188), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 255, 255), 1)
        cv2.putText(output, f"CONF: {evidence.confidence:.2f}  narrow-centre={evidence.line_center_x}  M: start/pause", (18, 216), cv2.FONT_HERSHEY_SIMPLEX, .44, (190, 190, 190), 1)
        cv2.putText(output, "Endpoint bar is never followed as a left/right path.", (18, 240), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 220, 255), 1)
        return output

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np

            with self._tuning_lock:
                initial_config = self.config
            if initial_config.use_hybrid:
                analyzer = HybridScanlineAnalyzer()
                self._set_status(running=True, state="ready", detail="Hybrid 扫描线 I 型识别运行中（骨架+交叉点预判）；按 M 开启自动行驶", variant="hybrid")
            else:
                analyzer = IShapeScanlineAnalyzer()
                self._set_status(running=True, state="ready", detail="扫描线 I 型识别运行中；按 M 开启自动行驶", mode="legacy")
            planner = IShapeTurnaroundPlanner(TurnaroundConfig(pivot_min_seconds=initial_config.pivot_min_seconds, pivot_max_seconds=initial_config.pivot_max_seconds))
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
                with self._tuning_lock:
                    config = self.config
                motor_text = "PAUSED"
                if self.gate.enabled():
                    # Only an explicitly M-enabled drive session may advance
                    # endpoint confirmation or pivot timing.
                    decision = planner.step(evidence, now)
                    if self._keeps_forward_motion(decision.state):
                        # A far junction is only a prediction.  Continue to
                        # follow the near line until the bar is marked and
                        # the stem-loss confirmation authorizes braking.
                        right_pwm, left_pwm = self._straight_pair(evidence, frame.shape[1], config)
                        self.controller.set_direct_drive(right_pwm, left_pwm)
                        self._motor_active, motor_text = True, f"P_STRAIGHT R={right_pwm} L={left_pwm}"
                    elif decision.state is TurnaroundState.PIVOT_180:
                        # Keep the same right-pivot sign convention as the main route mode.
                        self.controller.set_direct_drive(-config.pivot_pwm, config.pivot_pwm)
                        self._motor_active, motor_text = True, f"PIVOT_RIGHT R={-config.pivot_pwm} L={config.pivot_pwm}"
                    else:
                        self._stop_motor()
                        motor_text = "STOP_WAITING_FOR_ENDPOINT_CONFIRMATION"
                else:
                    self._stop_motor()
                    # Preview must not consume a future drive session.  A
                    # paused camera can remain parked on the bar indefinitely;
                    # the next M press must start a fresh confirmation window.
                    planner = IShapeTurnaroundPlanner(
                        TurnaroundConfig(
                            pivot_min_seconds=config.pivot_min_seconds,
                            pivot_max_seconds=config.pivot_max_seconds,
                        )
                    )
                    decision = planner.step(evidence, now)
                annotated = self._draw(cv2, frame, result, decision, motor_text)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                self._set_status(state=decision.state.value, detail=decision.reason, frame=frame_index, confidence=evidence.confidence, endpoint_detected=evidence.endpoint_detected, endpoint_y=evidence.endpoint_y, endpoint_width=evidence.endpoint_width, junction_detected=evidence.junction_detected, junction_y=evidence.junction_y, junction_arm_count=evidence.junction_arm_count, lookahead_x=evidence.lookahead_x, lookahead_y=evidence.lookahead_y, path_length_px=evidence.path_length_px, motor=motor_text)
                frame_index += 1
        except Exception as exc:
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            self._set_status(running=False)
