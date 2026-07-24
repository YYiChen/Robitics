"""Minimal line-present / line-lost safety state machine.

This deliberately has no corner, recovery, or turn behaviour.  A line that
disappears for the configured number of consecutive processed frames is an end
of route and the only valid next intent is STOP.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class LineIntent(str, Enum):
    STRAIGHT = "STRAIGHT"
    STOP = "STOP"


class LineObservation(Protocol):
    line_lost: bool
    confidence: float


@dataclass(frozen=True)
class LineStopConfig:
    minimum_confidence: float = 0.38
    line_lost_stop_frames: int = 5

    def __post_init__(self) -> None:
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if self.line_lost_stop_frames < 1:
            raise ValueError("line_lost_stop_frames must be at least one")


@dataclass(frozen=True)
class LineStopDecision:
    intent: LineIntent
    reason: str
    lost_frames: int


class StraightLineStopPlanner:
    """Follow only a visible line; latch STOP once the route ends."""

    def __init__(self, config: LineStopConfig = LineStopConfig()) -> None:
        self.config = config
        self._lost_frames = 0
        self._stopped = False

    def step(self, observation: LineObservation) -> LineStopDecision:
        if self._stopped:
            return LineStopDecision(LineIntent.STOP, "line_end_stop_latched", self._lost_frames)

        visible = not observation.line_lost and observation.confidence >= self.config.minimum_confidence
        if visible:
            self._lost_frames = 0
            return LineStopDecision(LineIntent.STRAIGHT, "following_visible_line", 0)

        self._lost_frames += 1
        if self._lost_frames >= self.config.line_lost_stop_frames:
            self._stopped = True
            return LineStopDecision(LineIntent.STOP, "line_end_confirmed", self._lost_frames)
        return LineStopDecision(LineIntent.STRAIGHT, "brief_line_loss_grace", self._lost_frames)
