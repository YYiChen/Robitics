"""Port-5000 validation adapter for the isolated four-endpoint pivot planner.

This mode drives only M1/M2 after the user presses M.  It intentionally does
not invoke M3/M4: reaching an endpoint stops the car at DEAL_CARD so the
turning loop can be proven before card hardware is added.
"""
from __future__ import annotations

from pathlib import Path
import sys
import time

from green_white_scanline_i_route import GreenWhiteScanlineIShapeRouteTracker
from scanline_i_route import ScanlineIRouteConfig


FOUR_ENDPOINT_EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "i_shape_four_endpoint_navigation_validation"
if str(FOUR_ENDPOINT_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(FOUR_ENDPOINT_EXPERIMENT))

from four_endpoint_planner import (  # noqa: E402
    DriveAction,
    Endpoint,
    FourEndpointPlanner,
    NavigationState,
    VisionObservation,
)


class FourEndpointValidationRouteTracker(GreenWhiteScanlineIShapeRouteTracker):
    """One-endpoint, M-gated live proof of visual 90-degree pivot control."""

    route_mode = "scanline_i_four_endpoint_green_white"
    route_variant = "four_endpoint_pivot_validation"
    route_ready_detail = "工字形四端点验证：默认去 [1]；只沿竖线并原地左转 90°，按 M 开始"
    validation_route = (Endpoint.TOP_LEFT,)

    @staticmethod
    def _motor_pair(action: DriveAction, evidence, frame_width: int, config: ScanlineIRouteConfig) -> tuple[int, int] | None:
        if action is DriveAction.FOLLOW_STEM:
            return FourEndpointValidationRouteTracker._straight_pair(evidence, frame_width, config)
        if action is DriveAction.PIVOT_LEFT_90:
            return config.pivot_pwm, -config.pivot_pwm
        if action is DriveAction.PIVOT_RIGHT_90:
            return -config.pivot_pwm, config.pivot_pwm
        return None

    @staticmethod
    def _vision_observation(evidence) -> VisionObservation:
        # During a pivot the planner requires an intervening blank/sideways
        # phase and then this bottom-centred, valid white route to reappear.
        return VisionObservation(
            junction_detected=evidence.junction_detected,
            forward_line_detected=(
                evidence.valid_line
                and evidence.confidence >= .55
                and evidence.line_center_x is not None
            ),
        )

    def _draw(self, cv2, frame, result, decision, motor_text: str):
        output = super()._draw(cv2, frame, result, decision, motor_text)
        target = "—" if decision.active_target is None else decision.active_target.value
        pending = "—" if decision.pending_heading is None else decision.pending_heading.value
        cv2.rectangle(output, (10, 232), (940, 252), (20, 20, 20), cv2.FILLED)
        cv2.putText(
            output,
            f"FOUR-ENDPOINT VALIDATION: target [{target}] heading={decision.heading.value} next={pending}; no crossbar drive, M3/M4 disabled.",
            (18, 248), cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 220, 255), 1,
        )
        return output

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np

            with self._tuning_lock:
                initial_config = self.config
            analyzer = self._create_analyzer(initial_config)
            self._open_run_log(initial_config)
            self._set_status(running=True, state="ready", **self._ready_status(initial_config))
            planner = FourEndpointPlanner(self.validation_route)
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
                observation = self._vision_observation(evidence)
                with self._tuning_lock:
                    config = self.config
                motor_text, commanded = "PAUSED", None
                if self.gate.enabled():
                    decision = planner.step(observation)
                    pair = self._motor_pair(decision.action, evidence, frame.shape[1], config)
                    if pair is None:
                        self._stop_motor()
                        motor_text = "DEAL_CARD_DISABLED_STOP" if decision.state is NavigationState.DEAL_CARD else "STOP"
                    else:
                        self.controller.set_direct_drive(*pair)
                        commanded = pair
                        self._motor_active = True
                        motor_text = f"{decision.action.value} R={pair[0]} L={pair[1]}"
                else:
                    self._stop_motor()
                    # A paused preview must not retain an old junction/pivot.
                    planner = FourEndpointPlanner(self.validation_route)
                    decision = planner.step(observation)
                annotated = self._draw(cv2, frame, result, decision, motor_text)
                ok, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                if self.gate.enabled() or frame_index % max(1, int(config.process_fps)) == 0:
                    self._write_run_log(evidence, decision, frame_index, now, motor_text, commanded)
                self._set_status(
                    state=decision.state.value, detail=decision.reason, frame=frame_index,
                    confidence=evidence.confidence, endpoint_detected=evidence.endpoint_detected,
                    endpoint_y=evidence.endpoint_y, endpoint_width=evidence.endpoint_width,
                    junction_detected=evidence.junction_detected, junction_y=evidence.junction_y,
                    junction_arm_count=evidence.junction_arm_count,
                    lookahead_x=evidence.lookahead_x, lookahead_y=evidence.lookahead_y,
                    path_length_px=evidence.path_length_px, motor=motor_text,
                    active_target=None if decision.active_target is None else decision.active_target.value,
                    heading=decision.heading.value,
                    pending_heading=None if decision.pending_heading is None else decision.pending_heading.value,
                )
                frame_index += 1
        except Exception as exc:
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            self._close_run_log()
            self._set_status(running=False)
