"""Shape-agnostic decisions for continuously visible taped paths."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class PathIntent(str, Enum):
    FOLLOW_PATH = "FOLLOW_PATH"
    STOP = "STOP"


class PathObservation(Protocol):
    line_lost: bool
    confidence: float
    lookahead_offset: float | None


@dataclass(frozen=True)
class ContinuousPathConfig:
    minimum_confidence: float = 0.38
    line_lost_stop_frames: int = 3
    line_lost_stop_seconds: float = 0.45

    def __post_init__(self) -> None:
        if not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if self.line_lost_stop_frames < 1:
            raise ValueError("line_lost_stop_frames must be positive")
        if self.line_lost_stop_seconds < 0:
            raise ValueError("line_lost_stop_seconds must be non-negative")


@dataclass(frozen=True)
class PathDecision:
    intent: PathIntent
    reason: str
    lost_frames: int


class ContinuousPathPlanner:
    """Follow any visible continuous line; do not assume a polygon or turn count."""

    def __init__(self, config: ContinuousPathConfig = ContinuousPathConfig()) -> None:
        self.config = config
        self._lost_frames = 0
        self._has_seen_line = False
        self._stopped = False
        self._loss_started_at: float | None = None

    def step(self, observation: PathObservation, now: float) -> PathDecision:
        visible = (
            not observation.line_lost
            and observation.confidence >= self.config.minimum_confidence
            and observation.lookahead_offset is not None
        )
        if visible:
            # Do not unexpectedly start a vehicle that was launched while it
            # could not see its route. A stop after genuine tracking, however,
            # may safely resume when inertia carries the camera back onto line.
            if self._stopped and not self._has_seen_line:
                return PathDecision(PathIntent.STOP, "startup_line_missing", 0)
            resumed = self._stopped
            self._stopped = False
            self._has_seen_line = True
            self._lost_frames = 0
            self._loss_started_at = None
            reason = "line_reacquired_resume_following" if resumed else "following_visible_lookahead_path"
            return PathDecision(PathIntent.FOLLOW_PATH, reason, 0)
        if self._stopped:
            return PathDecision(PathIntent.STOP, "line_lost_waiting_for_reacquire", self._lost_frames)
        if not self._has_seen_line:
            self._stopped = True
            return PathDecision(PathIntent.STOP, "startup_line_missing", 0)
        self._lost_frames += 1
        if self._loss_started_at is None:
            self._loss_started_at = now
        lost_seconds = now - self._loss_started_at
        if (
            self._lost_frames >= self.config.line_lost_stop_frames
            and lost_seconds >= self.config.line_lost_stop_seconds
        ):
            self._stopped = True
            return PathDecision(PathIntent.STOP, "line_lost_stop_waiting_for_reacquire", self._lost_frames)
        return PathDecision(PathIntent.FOLLOW_PATH, "brief_line_loss_keep_heading", self._lost_frames)
