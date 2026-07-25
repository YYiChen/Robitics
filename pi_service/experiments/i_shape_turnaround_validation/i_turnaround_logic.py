"""Vision-only state machine for an I-shaped route with turnaround bars."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class TurnaroundState(str, Enum):
    FOLLOW_STRAIGHT = "FOLLOW_STRAIGHT"
    PIVOT_180 = "PIVOT_180"
    STOP = "STOP"


@dataclass(frozen=True)
class TurnaroundConfig:
    end_confirm_frames: int = 3
    reacquire_confirm_frames: int = 3
    endpoint_margin_px: float = 45.0
    minimum_straight_span_ratio: float = 0.28
    minimum_straight_aspect: float = 1.35
    pivot_min_seconds: float = 1.60
    pivot_max_seconds: float = 5.00
    minimum_confidence: float = 0.45


@dataclass(frozen=True)
class RouteEvidence:
    confidence: float
    line_lost: bool
    lookahead_px: tuple[int, int] | None
    centerline_px: tuple[tuple[int, int], ...]
    marker_detected: bool
    marker_point_px: tuple[int, int] | None
    marker_branch_count: int
    frame_height: int

    @property
    def valid_line(self) -> bool:
        return not self.line_lost and self.lookahead_px is not None

    @property
    def span(self) -> tuple[float, float]:
        if len(self.centerline_px) < 2:
            return 0.0, 0.0
        xs, ys = zip(*self.centerline_px)
        return float(max(xs) - min(xs)), float(max(ys) - min(ys))


@dataclass(frozen=True)
class TurnaroundDecision:
    state: TurnaroundState
    reason: str
    end_frames: int
    reacquire_frames: int
    pivot_elapsed_seconds: float | None


class IShapeTurnaroundPlanner:
    """Treat transverse bars as endpoint evidence, never as a new guide line."""

    def __init__(self, config: TurnaroundConfig = TurnaroundConfig()) -> None:
        self.config = config
        self.state = TurnaroundState.FOLLOW_STRAIGHT
        self._end_frames = 0
        self._reacquire_frames = 0
        self._pivot_started_at: float | None = None

    def _is_endpoint(self, evidence: RouteEvidence) -> bool:
        if not evidence.valid_line or evidence.confidence < self.config.minimum_confidence:
            return False
        if not evidence.marker_detected or evidence.marker_point_px is None or evidence.marker_branch_count < 3:
            return False
        span_x, span_y = evidence.span
        if span_y < evidence.frame_height * self.config.minimum_straight_span_ratio:
            return False
        marker_y = evidence.marker_point_px[1]
        # The selected near-to-far route must end at the transverse mark. If
        # it extends substantially above the bar, this is a mid-route crossbar
        # rather than the end of the I-shaped guide line.
        end_y = evidence.centerline_px[-1][1] if evidence.centerline_px else evidence.frame_height
        return end_y >= marker_y - self.config.endpoint_margin_px

    def _is_reacquired_straight(self, evidence: RouteEvidence) -> bool:
        if not evidence.valid_line or evidence.confidence < self.config.minimum_confidence:
            return False
        # A transverse bar/X is explicitly rejected during pivot recovery.
        if evidence.marker_detected:
            return False
        span_x, span_y = evidence.span
        return span_y >= evidence.frame_height * self.config.minimum_straight_span_ratio and span_y >= max(1.0, span_x) * self.config.minimum_straight_aspect

    def step(self, evidence: RouteEvidence, now: float) -> TurnaroundDecision:
        if self.state is TurnaroundState.FOLLOW_STRAIGHT:
            self._end_frames = self._end_frames + 1 if self._is_endpoint(evidence) else 0
            if self._end_frames >= self.config.end_confirm_frames:
                self.state = TurnaroundState.PIVOT_180
                self._pivot_started_at = now
                self._reacquire_frames = 0
                return TurnaroundDecision(self.state, "endpoint_bar_and_longitudinal_line_terminated", self._end_frames, 0, 0.0)
            return TurnaroundDecision(self.state, "following_longitudinal_main_line", self._end_frames, 0, None)

        if self.state is TurnaroundState.PIVOT_180:
            started = self._pivot_started_at if self._pivot_started_at is not None else now
            elapsed = max(0.0, now - started)
            if elapsed >= self.config.pivot_max_seconds:
                self.state = TurnaroundState.STOP
                return TurnaroundDecision(self.state, "pivot_timeout_no_longitudinal_line", self._end_frames, self._reacquire_frames, elapsed)
            if elapsed < self.config.pivot_min_seconds:
                return TurnaroundDecision(self.state, "pivoting_before_180_time_window", self._end_frames, 0, elapsed)
            self._reacquire_frames = self._reacquire_frames + 1 if self._is_reacquired_straight(evidence) else 0
            if self._reacquire_frames >= self.config.reacquire_confirm_frames:
                self.state = TurnaroundState.FOLLOW_STRAIGHT
                self._end_frames = 0
                return TurnaroundDecision(self.state, "180_turn_complete_longitudinal_line_reacquired", 0, self._reacquire_frames, elapsed)
            return TurnaroundDecision(self.state, "pivoting_until_non_transverse_longitudinal_line", self._end_frames, self._reacquire_frames, elapsed)

        return TurnaroundDecision(self.state, "stopped_after_pivot_timeout", self._end_frames, self._reacquire_frames, None)
