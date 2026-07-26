"""I-shape turnaround state machine independent of OpenCV and hardware."""
from __future__ import annotations

from .models import (
    ScanlineEvidence,
    TurnaroundConfig,
    TurnaroundDecision,
    TurnaroundState,
)


class IShapeTurnaroundPlanner:
    def __init__(self, config: TurnaroundConfig = TurnaroundConfig()) -> None:
        self.config = config
        self.state = TurnaroundState.FOLLOW_STRAIGHT
        self._endpoint_frames = 0
        self._line_lost_frames = 0
        self._reacquire_frames = 0
        self._bar_marked_at: float | None = None
        self._brake_started_at: float | None = None
        self._pivot_started_at: float | None = None
        # ---- Hybrid ----
        self._junction_frames = 0
        # Preserve the most recent junction after it is confirmed.  A junction
        # can disappear while the vehicle is passing under the transverse bar;
        # the fast stem-loss decision must not lose that prior authorization.
        self._latched_junction_y: int | None = None
        self._latched_endpoint_y: int | None = None
        self._latched_frame_height = 0
        self._red_marker_bottom_armed = False
        self._red_marker_missing_frames = 0

    @property
    def red_exit_armed(self) -> bool:
        """Whether the confirmed red band has reached the camera near field."""
        return self._red_marker_bottom_armed

    def diagnostics(self) -> dict[str, object]:
        """Expose stable, read-only state for runtime telemetry.

        Keeping this projection here avoids having the web adapter reach into
        private counters and makes log-schema changes independent from the
        planner's internal attribute names.
        """
        return {
            "endpoint_frames": self._endpoint_frames,
            "line_lost_frames": self._line_lost_frames,
            "reacquire_frames": self._reacquire_frames,
            "junction_frames": self._junction_frames,
            "latched_junction_y_px": self._latched_junction_y,
            "latched_endpoint_y_px": self._latched_endpoint_y,
            "red_exit_armed": self._red_marker_bottom_armed,
            "red_missing_frames": self._red_marker_missing_frames,
            "fast_stem_loss_authorized": self._fast_stem_loss_authorized(),
        }

    def _observe_red_marker(self, evidence: ScanlineEvidence) -> bool:
        """Arm on a near red marker, then report its confirmed bottom-edge exit.

        This deliberately does not cause a turn by itself.  The caller only
        consumes it after the independent white transverse bar has put the
        planner in BAR_MARKED.
        """
        if not self.config.red_exit_enabled:
            return False
        if (
            evidence.red_marker_detected
            and evidence.red_marker_y is not None
            and evidence.frame_height > 0
        ):
            if evidence.red_marker_y >= evidence.frame_height * self.config.red_exit_arm_y_ratio:
                self._red_marker_bottom_armed = True
            self._red_marker_missing_frames = 0
            return False
        if not self._red_marker_bottom_armed:
            return False
        self._red_marker_missing_frames += 1
        return self._red_marker_missing_frames >= self.config.red_exit_confirm_frames

    def _latch_junction(self, evidence: ScanlineEvidence) -> None:
        if evidence.junction_detected and evidence.junction_y is not None and evidence.frame_height > 0:
            self._latched_junction_y = evidence.junction_y
            self._latched_frame_height = evidence.frame_height

    def _latch_endpoint(self, evidence: ScanlineEvidence) -> None:
        if evidence.endpoint_detected and evidence.endpoint_y is not None and evidence.frame_height > 0:
            self._latched_endpoint_y = evidence.endpoint_y
            self._latched_frame_height = evidence.frame_height

    def _fast_stem_loss_authorized(self) -> bool:
        return (
            (self._latched_junction_y is not None or self._latched_endpoint_y is not None)
            and self._latched_frame_height > 0
            and max(value for value in (self._latched_junction_y, self._latched_endpoint_y) if value is not None)
            >= self._latched_frame_height * self.config.early_junction_trigger_y_ratio
        )

    def _clear_early_prediction(self) -> None:
        self._junction_frames = 0
        self._latched_junction_y = None
        self._latched_endpoint_y = None
        self._latched_frame_height = 0
        self._red_marker_bottom_armed = False
        self._red_marker_missing_frames = 0

    def step(self, evidence: ScanlineEvidence, now: float) -> TurnaroundDecision:
        usable = evidence.valid_line and evidence.confidence >= self.config.minimum_confidence
        red_marker_exited_bottom = self._observe_red_marker(evidence)

        # ================================================================
        # FOLLOW_STRAIGHT
        # ================================================================
        if self.state is TurnaroundState.FOLLOW_STRAIGHT:
            # ---- Junction early prediction (hybrid) ----
            self._latch_junction(evidence)
            self._junction_frames = (
                self._junction_frames + 1
                if usable and evidence.junction_detected
                else 0
            )
            if self._junction_frames >= self.config.junction_confirm_frames:
                self.state = TurnaroundState.EARLY_BAR_PREDICTED
                self._endpoint_frames = 0
                self._line_lost_frames = 0
                return TurnaroundDecision(
                    self.state,
                    f"junction_at_y={evidence.junction_y}_early_bar_predicted",
                    self._endpoint_frames, 0, 0, None,
                    junction_frames=self._junction_frames,
                )

            # ---- Standard endpoint detection (legacy + hybrid fallback) ----
            self._endpoint_frames = (
                self._endpoint_frames + 1
                if usable and evidence.endpoint_detected
                else 0
            )
            if self._endpoint_frames >= self.config.endpoint_confirm_frames:
                self.state = TurnaroundState.BAR_MARKED
                self._bar_marked_at = now
                self._line_lost_frames = 0
                return TurnaroundDecision(
                    self.state,
                    "lower_transverse_bar_marked_follow_until_stem_lost",
                    self._endpoint_frames, 0, 0, None,
                )
            return TurnaroundDecision(
                self.state,
                "following_near_anchored_longitudinal_line",
                self._endpoint_frames, 0, 0, None,
            )

        # ================================================================
        # EARLY_BAR_PREDICTED (hybrid): junction seen in far field.
        # Keep driving straight; transition to BAR_MARKED when either
        # endpoint_detected fires or line is lost (stem disappeared).
        # False alarm recovery if junction disappears.
        # ================================================================
        if self.state is TurnaroundState.EARLY_BAR_PREDICTED:
            self._latch_junction(evidence)
            self._latch_endpoint(evidence)
            # Confirm: if the bar arrives at bar_rows while we're waiting.
            self._endpoint_frames = (
                self._endpoint_frames + 1
                if usable and evidence.endpoint_detected
                else 0
            )
            # The junction already supplied a multi-frame early warning, so
            # one lower/middle bar frame is sufficient to mark the endpoint.
            if self._endpoint_frames >= 1:
                self.state = TurnaroundState.BAR_MARKED
                self._bar_marked_at = now
                self._line_lost_frames = 0
                return TurnaroundDecision(
                    self.state,
                    "early_prediction_confirmed_by_bar_rows_endpoint",
                    self._endpoint_frames, 0, 0, None,
                )

            # Confirm: stem lost (vehicle crossed the bar before bar_rows
            # detected it — can happen with fast approach).
            self._line_lost_frames = (
                self._line_lost_frames + 1 if evidence.line_lost else 0
            )
            if self._line_lost_frames >= self.config.line_lost_confirm_frames:
                self.state = TurnaroundState.BRAKE_BEFORE_PIVOT
                self._brake_started_at = now
                return TurnaroundDecision(
                    self.state,
                    "stem_lost_from_early_prediction_braking",
                    self._endpoint_frames, self._line_lost_frames, 0, 0.0,
                )

            # False alarm check: junction disappeared and no bar confirmed.
            if not evidence.junction_detected and not evidence.endpoint_detected:
                self._junction_frames = max(0, self._junction_frames - 1)
                if self._junction_frames == 0:
                    self.state = TurnaroundState.FOLLOW_STRAIGHT
                    self._endpoint_frames = 0
                    self._line_lost_frames = 0
                    self._clear_early_prediction()
                    return TurnaroundDecision(
                        self.state,
                        "early_prediction_false_alarm_junction_lost",
                        0, 0, 0, None,
                    )

            return TurnaroundDecision(
                self.state,
                f"early_bar_predicted_waiting_for_stem_loss_or_endpoint_confirm_junction_y={evidence.junction_y}",
                self._endpoint_frames, self._line_lost_frames, 0, None,
            )

        # ================================================================
        # BAR_MARKED
        # ================================================================
        if self.state is TurnaroundState.BAR_MARKED:
            marked_at = self._bar_marked_at if self._bar_marked_at is not None else now
            if now - marked_at >= self.config.bar_mark_timeout_seconds:
                self.state = TurnaroundState.FOLLOW_STRAIGHT
                self._endpoint_frames = 0
                self._line_lost_frames = 0
                self._clear_early_prediction()
                return TurnaroundDecision(
                    self.state, "bar_mark_timeout_returning_to_follow", 0, 0, 0, None,
                )
            # Fixed green-floor course: the white bar independently confirmed
            # the endpoint, and the red band has travelled through the near
            # field then left the bottom of the image.  On this calibrated
            # layout that is the immediate physical stop/pivot moment.
            if red_marker_exited_bottom:
                self.state = TurnaroundState.BRAKE_BEFORE_PIVOT
                self._brake_started_at = now
                return TurnaroundDecision(
                    self.state,
                    "confirmed_white_bar_red_marker_exited_bottom_braking",
                    self._endpoint_frames, self._line_lost_frames, 0, 0.0,
                )
            # A missing near longitudinal stem means the car has passed the
            # bar.  Do not require the bar itself to remain visible.
            self._line_lost_frames = (
                self._line_lost_frames + 1 if evidence.line_lost else 0
            )
            required_lost_frames = (
                self.config.early_line_lost_confirm_frames
                if self._fast_stem_loss_authorized()
                else self.config.line_lost_confirm_frames
            )
            if self._line_lost_frames >= required_lost_frames:
                self.state = TurnaroundState.BRAKE_BEFORE_PIVOT
                self._brake_started_at = now
                return TurnaroundDecision(
                    self.state,
                    "longitudinal_stem_lost_after_bar_fast_braking"
                    if required_lost_frames == self.config.early_line_lost_confirm_frames
                    else "longitudinal_stem_lost_after_bar_braking",
                    self._endpoint_frames, self._line_lost_frames, 0, 0.0,
                )
            return TurnaroundDecision(
                self.state,
                "bar_marked_following_until_stem_lost",
                self._endpoint_frames, self._line_lost_frames, 0, None,
            )

        # ================================================================
        # BRAKE_BEFORE_PIVOT
        # ================================================================
        if self.state is TurnaroundState.BRAKE_BEFORE_PIVOT:
            started = self._brake_started_at if self._brake_started_at is not None else now
            if now - started < self.config.brake_seconds:
                return TurnaroundDecision(
                    self.state,
                    "braking_before_right_pivot",
                    self._endpoint_frames, self._line_lost_frames, 0, None,
                )
            self.state = TurnaroundState.PIVOT_180
            self._pivot_started_at = now
            self._reacquire_frames = 0
            return TurnaroundDecision(
                self.state,
                "brake_complete_starting_right_pivot",
                self._endpoint_frames, self._line_lost_frames, 0, 0.0,
            )

        # ================================================================
        # PIVOT_180
        # ================================================================
        if self.state is TurnaroundState.PIVOT_180:
            started = self._pivot_started_at if self._pivot_started_at is not None else now
            elapsed = max(0.0, now - started)
            if elapsed >= self.config.pivot_max_seconds:
                self.state = TurnaroundState.STOP
                return TurnaroundDecision(
                    self.state,
                    "pivot_timeout_without_longitudinal_reacquire",
                    self._endpoint_frames, self._line_lost_frames,
                    self._reacquire_frames, elapsed,
                )
            if elapsed < self.config.pivot_min_seconds:
                return TurnaroundDecision(
                    self.state,
                    "pivoting_minimum_time",
                    self._endpoint_frames, self._line_lost_frames, 0, elapsed,
                )
            self._reacquire_frames = (
                self._reacquire_frames + 1
                if usable and not evidence.endpoint_detected
                else 0
            )
            if self._reacquire_frames >= self.config.reacquire_confirm_frames:
                self.state = TurnaroundState.FOLLOW_STRAIGHT
                self._endpoint_frames = 0
                self._clear_early_prediction()
                return TurnaroundDecision(
                    self.state,
                    "longitudinal_line_reacquired",
                    0, self._line_lost_frames, self._reacquire_frames, elapsed,
                )
            return TurnaroundDecision(
                self.state,
                "pivoting_until_longitudinal_reacquire",
                self._endpoint_frames, self._line_lost_frames,
                self._reacquire_frames, elapsed,
            )

        # ================================================================
        # STOP
        # ================================================================
        return TurnaroundDecision(
            self.state,
            "stopped_after_pivot_timeout",
            self._endpoint_frames, self._line_lost_frames,
            self._reacquire_frames, None,
        )
