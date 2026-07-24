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

    def __post_init__(self) -> None:
        if not 0 <= self.minimum_confidence <= 1:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if self.line_lost_stop_frames < 1:
            raise ValueError("line_lost_stop_frames must be positive")


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

    def step(self, observation: PathObservation) -> PathDecision:
        visible = (
            not observation.line_lost
            and observation.confidence >= self.config.minimum_confidence
            and observation.lookahead_offset is not None
        )
        if self._stopped:
            return PathDecision(PathIntent.STOP, "line_lost_stop_latched", self._lost_frames)
        if visible:
            self._has_seen_line = True
            self._lost_frames = 0
            return PathDecision(PathIntent.FOLLOW_PATH, "following_visible_lookahead_path", 0)
        if not self._has_seen_line:
            self._stopped = True
            return PathDecision(PathIntent.STOP, "startup_line_missing", 0)
        self._lost_frames += 1
        if self._lost_frames >= self.config.line_lost_stop_frames:
            self._stopped = True
            return PathDecision(PathIntent.STOP, "line_lost_stop", self._lost_frames)
        return PathDecision(PathIntent.FOLLOW_PATH, "brief_line_loss_keep_heading", self._lost_frames)
