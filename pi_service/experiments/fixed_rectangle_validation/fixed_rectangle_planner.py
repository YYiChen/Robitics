"""Fixed clockwise rectangle state machine with no branch inference."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class RectangleIntent(str, Enum):
    FOLLOW_LINE = "FOLLOW_LINE"
    FORWARD_APPROACH = "FORWARD_APPROACH"
    PIVOT_RIGHT = "PIVOT_RIGHT"
    FORWARD_REACQUIRE = "FORWARD_REACQUIRE"
    STOP = "STOP"


class RectangleState(str, Enum):
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    TURN_RIGHT = "TURN_RIGHT"
    REACQUIRE = "REACQUIRE"
    COMPLETE = "COMPLETE"
    LOST = "LOST"


class LineObservation(Protocol):
    line_lost: bool
    confidence: float


@dataclass(frozen=True)
class FixedRectangleConfig:
    minimum_confidence: float = 0.38
    line_lost_corner_frames: int = 5
    corner_forward_seconds: float = 0.20
    right_turn_seconds: float = 0.30
    reacquire_frames: int = 3
    reacquire_timeout_seconds: float = 0.80
    corners_to_complete: int = 4

    def __post_init__(self) -> None:
        if not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum confidence must be in [0, 1]")
        if self.line_lost_corner_frames < 1 or self.reacquire_frames < 1:
            raise ValueError("frame counts must be positive")
        if min(self.corner_forward_seconds, self.right_turn_seconds, self.reacquire_timeout_seconds) < 0:
            raise ValueError("phase durations must be non-negative")
        if self.corners_to_complete < 1:
            raise ValueError("corners_to_complete must be at least one")


@dataclass(frozen=True)
class RectangleDecision:
    intent: RectangleIntent
    state: RectangleState
    reason: str
    corner_count: int
    lost_frames: int


class FixedClockwiseRectanglePlanner:
    """A calibrated right-turn sequence for a known rectangular route."""

    def __init__(self, config: FixedRectangleConfig = FixedRectangleConfig()) -> None:
        self.config = config
        self.state = RectangleState.FOLLOW
        self._lost_frames = 0
        self._reacquired_frames = 0
        self._phase_started_at = 0.0
        self._corner_count = 0
        self._has_seen_line = False

    def _visible(self, observation: LineObservation) -> bool:
        return not observation.line_lost and observation.confidence >= self.config.minimum_confidence

    def _decision(self, intent: RectangleIntent, reason: str) -> RectangleDecision:
        return RectangleDecision(intent, self.state, reason, self._corner_count, self._lost_frames)

    def step(self, observation: LineObservation, now: float) -> RectangleDecision:
        visible = self._visible(observation)
        if self.state in (RectangleState.COMPLETE, RectangleState.LOST):
            return self._decision(RectangleIntent.STOP, "route_complete" if self.state is RectangleState.COMPLETE else "new_line_not_found")

        if self.state is RectangleState.FOLLOW:
            if visible:
                self._has_seen_line = True
                self._lost_frames = 0
                return self._decision(RectangleIntent.FOLLOW_LINE, "following_visible_line")
            if not self._has_seen_line:
                self.state = RectangleState.LOST
                return self._decision(RectangleIntent.STOP, "startup_line_missing")
            self._lost_frames += 1
            if self._lost_frames < self.config.line_lost_corner_frames:
                return self._decision(RectangleIntent.FOLLOW_LINE, "brief_line_loss_grace")
            self.state = RectangleState.APPROACH
            self._phase_started_at = now
            return self._decision(RectangleIntent.FORWARD_APPROACH, "line_end_confirmed_forward_to_corner")

        if self.state is RectangleState.APPROACH:
            if now - self._phase_started_at < self.config.corner_forward_seconds:
                return self._decision(RectangleIntent.FORWARD_APPROACH, "camera_to_vehicle_corner_offset")
            self.state = RectangleState.TURN_RIGHT
            self._phase_started_at = now
            return self._decision(RectangleIntent.PIVOT_RIGHT, "fixed_right_turn_started")

        if self.state is RectangleState.TURN_RIGHT:
            if now - self._phase_started_at < self.config.right_turn_seconds:
                return self._decision(RectangleIntent.PIVOT_RIGHT, "fixed_right_turn_in_progress")
            self._corner_count += 1
            if self._corner_count >= self.config.corners_to_complete:
                self.state = RectangleState.COMPLETE
                return self._decision(RectangleIntent.STOP, "four_corners_complete")
            self.state = RectangleState.REACQUIRE
            self._phase_started_at = now
            self._reacquired_frames = 0
            return self._decision(RectangleIntent.FORWARD_REACQUIRE, "turn_complete_searching_new_line")

        # After the fixed rotation, gently move forward until the new straight
        # line is stable. A timeout is safer than turning again blindly.
        if visible:
            self._reacquired_frames += 1
            if self._reacquired_frames >= self.config.reacquire_frames:
                self.state = RectangleState.FOLLOW
                self._lost_frames = 0
                return self._decision(RectangleIntent.FOLLOW_LINE, "new_line_confirmed")
        else:
            self._reacquired_frames = 0
        if now - self._phase_started_at >= self.config.reacquire_timeout_seconds:
            self.state = RectangleState.LOST
            return self._decision(RectangleIntent.STOP, "new_line_reacquire_timeout")
        return self._decision(RectangleIntent.FORWARD_REACQUIRE, "waiting_for_new_line")
