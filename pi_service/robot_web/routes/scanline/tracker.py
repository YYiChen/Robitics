"""Port-5000 adapter for the isolated scanline I-shape turnaround algorithm."""
from __future__ import annotations

from dataclasses import replace
import json
import threading
import time
import traceback
from uuid import uuid4

from .config import (
    SCANLINE_TUNING_FIELDS,
    ScanlineIRouteConfig,
    load_scanline_tuning_config,
)
from .control import straight_control, straight_pair
from .logging import RUN_LOG_SCHEMA_VERSION, ScanlineLoggingMixin
from .perception_planner import (
    HybridScanlineAnalyzer,
    HybridScanlineConfig,
    IShapeScanlineAnalyzer,
    IShapeTurnaroundPlanner,
    TurnaroundConfig,
    TurnaroundState,
)


FORWARD_TRACKING_STATES = frozenset((
    TurnaroundState.FOLLOW_STRAIGHT,
    TurnaroundState.EARLY_BAR_PREDICTED,
    TurnaroundState.BAR_MARKED,
))


class ScanlineIShapeRouteTracker(ScanlineLoggingMixin):
    """Keep the main console's camera, M gate, and preview while reusing the isolated detector."""
    route_mode = "scanline_i"
    route_variant = "hybrid"
    route_ready_detail = "Hybrid 扫描线 I 型识别运行中（骨架+交叉点预判）；按 M 开启自动行驶"

    def __init__(self, controller, camera, publisher, gate, config: ScanlineIRouteConfig = ScanlineIRouteConfig()) -> None:
        self.controller, self.camera, self.publisher, self.gate, self.config = controller, camera, publisher, gate, config
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._tuning_lock = threading.RLock()
        self._motor_active = False
        self._planner: IShapeTurnaroundPlanner | None = None
        self._run_log = None
        self._log_lock = threading.Lock()
        self._run_id: str | None = None
        self._drive_session_id: str | None = None
        self._config_revision = 0
        self._last_frame_index: int | None = None
        self._previous_logged_state: str | None = None
        self._session_started = False
        self._session_ended = False
        self._status = {"available": True, "running": False, "enabled": False, "mode": self.route_mode, "state": "starting", "detail": "正在启动扫描线 I 型识别", "frame": 0, "confidence": None}

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
            status["config_revision"] = self._config_revision
        status["logging"] = {
            "schema_version": RUN_LOG_SCHEMA_VERSION,
            "run_id": self._run_id,
            "drive_session_id": self._drive_session_id,
            "active": self._run_log is not None,
        }
        return status

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled:
            self._stop_motor()
            self._write_event(
                "drive_disabled",
                frame_id=self._last_frame_index,
                drive_session_id=self._drive_session_id,
                reason_code="USER_PAUSED",
            )
            self._drive_session_id = None
        else:
            self._drive_session_id = f"drive-{uuid4().hex[:8]}"
            self._write_event(
                "drive_enabled",
                frame_id=self._last_frame_index,
                drive_session_id=self._drive_session_id,
                reason_code="USER_ENABLED",
            )
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
            previous = self.config
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
            self._config_revision += 1
            revision = self._config_revision
            # Apply state-machine thresholds immediately without resetting the
            # current endpoint or pivot progress.
            if self._planner is not None:
                self._planner.config = self._planner_config(updated)
        applied_changes = {
            field: {"old": getattr(previous, field), "new": getattr(updated, field)}
            for field in changes
            if getattr(previous, field) != getattr(updated, field)
        }
        self._write_event(
            "tuning_changed",
            frame_id=self._last_frame_index,
            config_revision=revision,
            drive_enabled=self.gate.enabled(),
            changes=applied_changes,
        )
        self._set_status(detail="扫描线 I 型直行与掉头参数已实时应用并保存")
        return self.status_dict()

    @staticmethod
    def _straight_control(evidence, frame_width: int, config: ScanlineIRouteConfig) -> tuple[tuple[int, int], dict[str, object]]:
        return straight_control(evidence, frame_width, config)

    @staticmethod
    def _straight_pair(evidence, frame_width: int, config: ScanlineIRouteConfig) -> tuple[int, int]:
        return straight_pair(evidence, frame_width, config)

    @staticmethod
    def _keeps_forward_motion(state: TurnaroundState) -> bool:
        """A far junction is a prediction only; it must never brake the car."""
        return state in FORWARD_TRACKING_STATES

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now()
        self._motor_active = False

    @staticmethod
    def _planner_config(config: ScanlineIRouteConfig) -> TurnaroundConfig:
        return TurnaroundConfig(
            pivot_min_seconds=config.pivot_min_seconds,
            pivot_max_seconds=config.pivot_max_seconds,
            bar_mark_timeout_seconds=config.bar_mark_timeout_seconds,
            early_junction_trigger_y_ratio=config.early_junction_trigger_y_ratio,
            early_line_lost_confirm_frames=config.early_line_lost_confirm_frames,
            red_exit_arm_y_ratio=config.red_exit_arm_y_ratio,
        )

    def _create_analyzer(self, config: ScanlineIRouteConfig):
        return HybridScanlineAnalyzer() if config.use_hybrid else IShapeScanlineAnalyzer()

    def _ready_status(self, config: ScanlineIRouteConfig) -> dict[str, str]:
        if config.use_hybrid:
            return {"mode": self.route_mode, "variant": self.route_variant, "detail": self.route_ready_detail}
        return {"mode": self.route_mode, "variant": "legacy", "detail": "扫描线 I 型识别运行中；按 M 开启自动行驶"}

    def _draw(self, cv2, frame, result, decision, motor_text: str):
        candidate_mask = getattr(self, "_visual_tape_candidate_mask", None)
        output = frame.copy()
        # Pale yellow is every white tape candidate retained inside the green
        # course.  It gives a truthful visual answer when a near horizontal
        # bar is deliberately withheld from steering control.
        if candidate_mask is not None and candidate_mask.shape == frame.shape[:2] and candidate_mask.any():
            candidate = output.copy()
            candidate[candidate_mask > 0] = (90, 210, 255)
            output = cv2.addWeighted(output, .72, candidate, .28, 0)
        # Yellow, not grayscale: this is the selected near-anchored route
        # component used by control, so it remains legible on a grey floor.
        yellow_route = output.copy()
        yellow_route[result.component_mask > 0] = (0, 220, 255)
        output = cv2.addWeighted(output, .62, yellow_route, .38, 0)
        course_field = getattr(self, "_visual_course_field", None)
        if course_field is not None and course_field.shape == output.shape[:2] and course_field.any():
            contours, _ = cv2.findContours(course_field, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Red outline = the actual green-course area where white tape is
            # permitted to become a recognition candidate.
            cv2.drawContours(output, contours, -1, (0, 0, 255), 2, cv2.LINE_AA)
        red_marker_mask = getattr(self, "_visual_red_marker_mask", None)
        if red_marker_mask is not None and red_marker_mask.shape == output.shape[:2] and red_marker_mask.any():
            red_contours, _ = cv2.findContours(red_marker_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, red_contours, -1, (0, 0, 255), 3, cv2.LINE_AA)
        tape_fit_line = getattr(self, "_visual_tape_fit_line", None)
        if tape_fit_line is not None:
            # Black = permissive, diagnostic-only tape fit; it never commands
            # the motors and is intentionally distinct from yellow control.
            cv2.line(output, tape_fit_line[0], tape_fit_line[1], (0, 0, 0), 3, cv2.LINE_AA)
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
        if evidence.red_marker_detected and evidence.red_marker_y is not None:
            cv2.line(output, (0, evidence.red_marker_y), (output.shape[1] - 1, evidence.red_marker_y), (0, 0, 255), 2)
        color = (0, 220, 0) if self.gate.enabled() else (0, 180, 255)
        # Keep the debug panel at the top so it never covers the approaching
        # tape, red band, or junction in the lower driving field.
        cv2.rectangle(output, (10, 10), (940, 184), (20, 20, 20), cv2.FILLED)
        cv2.putText(output, f"SCANLINE I-TURN: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
        cv2.putText(output, f"STATE: {decision.state.value}  {decision.reason}", (18, 66), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
        cv2.putText(output, f"BAR: {evidence.endpoint_detected} y={evidence.endpoint_y} w={evidence.endpoint_width}  JUNCTION: {evidence.junction_detected} y={evidence.junction_y} arms={evidence.junction_arm_count}", (18, 94), cv2.FONT_HERSHEY_SIMPLEX, .46, (100, 220, 255), 1)
        cv2.putText(output, f"RED BAND: {evidence.red_marker_detected} y={evidence.red_marker_y} span={evidence.red_marker_span}", (18, 116), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 80, 255), 1)
        cv2.putText(output, f"LOOKAHEAD: ({evidence.lookahead_x}, {evidence.lookahead_y}) path={evidence.path_length_px}px  MOTOR: {motor_text}", (18, 138), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 255, 255), 1)
        cv2.putText(output, f"CONF: {evidence.confidence:.2f}  narrow-centre={evidence.line_center_x}  M: start/pause", (18, 160), cv2.FONT_HERSHEY_SIMPLEX, .44, (190, 190, 190), 1)
        cv2.putText(output, "Red pre-authorizes; white endpoint and stem loss remain mandatory.", (18, 182), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 220, 255), 1)
        return output

    def _run(self) -> None:
        session_outcome = "stopped"
        session_reason = "SERVICE_STOPPED"
        try:
            import cv2
            import numpy as np

            with self._tuning_lock:
                initial_config = self.config
            analyzer = self._create_analyzer(initial_config)
            self._set_status(running=True, state="ready", **self._ready_status(initial_config))
            planner = IShapeTurnaroundPlanner(self._planner_config(initial_config))
            with self._tuning_lock:
                self._planner = planner
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
                with self._tuning_lock:
                    config = self.config
                if self._run_log is None:
                    self._open_run_log(config, frame.shape)
                result = analyzer.analyze(frame)
                self._visual_course_field = getattr(analyzer, "course_field_mask", None)
                self._visual_tape_candidate_mask = getattr(analyzer, "tape_candidate_mask", None)
                self._visual_red_marker_mask = getattr(analyzer, "red_marker_mask", None)
                self._visual_tape_fit_line = getattr(analyzer, "tape_fit_line", None)
                evidence = result.evidence
                motor_text = "PAUSED"
                commanded: tuple[int, int] | None = None
                control: dict[str, object] = {"mode": "PAUSED"}
                if self.gate.enabled():
                    # Only an explicitly M-enabled drive session may advance
                    # endpoint confirmation or pivot timing.
                    decision = planner.step(evidence, now)
                    if self._keeps_forward_motion(decision.state):
                        # A far junction is only a prediction.  Continue to
                        # follow the near line until the bar is marked and
                        # the stem-loss confirmation authorizes braking.
                        (right_pwm, left_pwm), control = self._straight_control(evidence, frame.shape[1], config)
                        self.controller.set_direct_drive(right_pwm, left_pwm)
                        commanded = (right_pwm, left_pwm)
                        self._motor_active, motor_text = True, f"P_STRAIGHT R={right_pwm} L={left_pwm}"
                    elif decision.state is TurnaroundState.PIVOT_180:
                        # Keep the same right-pivot sign convention as the main route mode.
                        self.controller.set_direct_drive(-config.pivot_pwm, config.pivot_pwm)
                        commanded = (-config.pivot_pwm, config.pivot_pwm)
                        control = {
                            "mode": "PIVOT_CONTINUOUS",
                            "pivot_elapsed_seconds": decision.pivot_elapsed_seconds,
                            "commanded_right_pwm": commanded[0],
                            "commanded_left_pwm": commanded[1],
                        }
                        self._motor_active, motor_text = True, f"PIVOT_RIGHT R={-config.pivot_pwm} L={config.pivot_pwm}"
                    else:
                        self._stop_motor()
                        motor_text = "STOP_WAITING_FOR_ENDPOINT_CONFIRMATION"
                        control = {"mode": "BRAKE_OR_SAFE_STOP", "reason_code": self._reason_code(decision.reason)}
                else:
                    self._stop_motor()
                    # Preview must not consume a future drive session.  A
                    # paused camera can remain parked on the bar indefinitely;
                    # the next M press must start a fresh confirmation window.
                    planner = IShapeTurnaroundPlanner(self._planner_config(config))
                    with self._tuning_lock:
                        self._planner = planner
                    decision = planner.step(evidence, now)
                annotated = self._draw(cv2, frame, result, decision, motor_text)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                if self.gate.enabled() or frame_index % max(1, int(config.process_fps)) == 0:
                    self._write_run_log(evidence, decision, planner, frame_index, now, motor_text, commanded, control, frame.shape)
                self._set_status(state=decision.state.value, detail=decision.reason, frame=frame_index, confidence=evidence.confidence, endpoint_detected=evidence.endpoint_detected, endpoint_y=evidence.endpoint_y, endpoint_width=evidence.endpoint_width, junction_detected=evidence.junction_detected, junction_y=evidence.junction_y, junction_arm_count=evidence.junction_arm_count, red_marker_detected=evidence.red_marker_detected, red_marker_y=evidence.red_marker_y, red_marker_span=evidence.red_marker_span, lookahead_x=evidence.lookahead_x, lookahead_y=evidence.lookahead_y, path_length_px=evidence.path_length_px, motor=motor_text)
                self._last_frame_index = frame_index
                frame_index += 1
        except Exception as exc:
            session_outcome = "error"
            session_reason = "UNHANDLED_EXCEPTION"
            self._write_event(
                "error",
                frame_id=self._last_frame_index,
                error_type=type(exc).__name__,
                message=str(exc),
                traceback=traceback.format_exc(),
                state=self._status.get("state"),
            )
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            self._close_run_log(outcome=session_outcome, reason_code=session_reason)
            with self._tuning_lock:
                self._planner = None
            self._set_status(running=False)

