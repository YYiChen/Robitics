"""Offline intent planner for a clockwise rectangular guide-line route.

This module deliberately has no OpenCV, camera, serial, or motor dependency.
It consumes the stable fields emitted by the OpenCV line detector and describes
only the next high-level intent: continue straight, begin a right turn, or stop.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class LineEvidence(Protocol):
    """The subset of ``track_line.LineObservation`` needed by the planner."""

    offset: float | None
    heading: float | None
    confidence: float
    line_lost: bool


class RouteState(str, Enum):
    FOLLOW = "FOLLOW"
    RIGHT_CORNER_ARMED = "RIGHT_CORNER_ARMED"
    APPROACHING_RIGHT_CORNER = "APPROACHING_RIGHT_CORNER"
    TURNING_RIGHT = "TURNING_RIGHT"
    LOST = "LOST"


class RouteIntent(str, Enum):
    STRAIGHT = "STRAIGHT"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"


@dataclass(frozen=True)
class PlannerDecision:
    """A motor-independent result suitable for an overlay or future tracker."""

    intent: RouteIntent
    state: RouteState
    reason: str


@dataclass(frozen=True)
class RectanglePlannerConfig:
    """Initial thresholds; tune them from recordings of the mounted camera."""

    minimum_confidence: float = 0.55
    right_heading_threshold: float = 0.35
    right_offset_threshold: float = 0.12
    corner_confirmation_frames: int = 2
    corner_disarm_frames: int = 4
    # At the default 10 FPS this is a 0.6 s blind, slow-forward approach.
    # It compensates for a forward-facing camera seeing the old edge disappear
    # before the wheelbase has reached the physical corner.
    missing_before_turn: int = 6
    reacquire_frames: int = 3
    max_turn_frames: int = 100
    fixed_right_turn_on_line_end: bool = True


class ClockwiseRectanglePlanner:
    """Recognise a fixed clockwise rectangle's right-angle corners.

    A right turn is normally *armed* while the far ROI already bends right,
    then emitted only after the old line disappears.  On this project's
    fixed, no-branch clockwise rectangle, an unarmed line end is also treated
    as a right corner by default.  This fallback prevents a missed branch
    detector from turning into an unsafe stop at every physical corner.
    """

    def __init__(self, config: RectanglePlannerConfig = RectanglePlannerConfig()) -> None:
        self.config = config
        self.state = RouteState.FOLLOW
        self._right_evidence_frames = 0
        self._corner_absence_frames = 0
        self._missing_frames = 0
        self._turn_frames = 0
        self._reacquired_frames = 0

    def step(
        self,
        observation: LineEvidence,
        *,
        right_corner_ahead: bool = False,
        new_line_ready: bool = False,
    ) -> PlannerDecision:
        """Return the route intent for one detector observation.

        ``right_corner_ahead`` is a geometric branch detector's explicit
        evidence. It takes precedence over the weaker far-band heading cue.
        """
        if self._usable(observation):
            return self._on_line(
                observation,
                right_corner_ahead=right_corner_ahead,
                new_line_ready=new_line_ready,
            )
        return self._on_line_lost()

    def _usable(self, observation: LineEvidence) -> bool:
        return (
            not observation.line_lost
            and observation.offset is not None
            and observation.heading is not None
            and observation.confidence >= self.config.minimum_confidence
        )

    def _on_line(
        self,
        observation: LineEvidence,
        *,
        right_corner_ahead: bool,
        new_line_ready: bool,
    ) -> PlannerDecision:
        self._missing_frames = 0

        if self.state is RouteState.TURNING_RIGHT:
            if not new_line_ready:
                return self._decision(RouteIntent.TURN_RIGHT, "turning_until_new_line")
            self._reacquired_frames += 1
            if self._reacquired_frames < self.config.reacquire_frames:
                return self._decision(RouteIntent.TURN_RIGHT, "reacquiring_new_edge")
            self._reset_follow()
            return self._decision(RouteIntent.STRAIGHT, "new_edge_confirmed")

        if self.state is RouteState.APPROACHING_RIGHT_CORNER:
            self._reset_follow()
            return self._decision(RouteIntent.STRAIGHT, "line_reacquired_before_turn")

        if self.state is RouteState.LOST:
            self._reset_follow()
            return self._decision(RouteIntent.STOP, "line_reappeared_after_safety_stop")

        corner_evidence = right_corner_ahead or self._right_corner_evidence(observation)
        if corner_evidence:
            self._right_evidence_frames += 1
            self._corner_absence_frames = 0
        else:
            if self.state is RouteState.RIGHT_CORNER_ARMED:
                self._corner_absence_frames += 1
                if self._corner_absence_frames >= self.config.corner_disarm_frames:
                    self.state = RouteState.FOLLOW
                    self._right_evidence_frames = 0
                    self._corner_absence_frames = 0
                else:
                    return self._decision(RouteIntent.STRAIGHT, "right_corner_latched")
            else:
                self._right_evidence_frames = 0

        if self._right_evidence_frames >= self.config.corner_confirmation_frames:
            self.state = RouteState.RIGHT_CORNER_ARMED
            return self._decision(RouteIntent.STRAIGHT, "right_corner_seen_ahead")
        return self._decision(RouteIntent.STRAIGHT, "following_visible_line")

    def _right_corner_evidence(self, observation: LineEvidence) -> bool:
        # Positive heading means the far line sits to the right of the near line.
        return (
            observation.heading >= self.config.right_heading_threshold
            and observation.offset >= -self.config.right_offset_threshold
        )

    def _on_line_lost(self) -> PlannerDecision:
        self._missing_frames += 1

        if self.state is RouteState.TURNING_RIGHT:
            return self._turn_or_stop("turning_searching_new_edge")

        expects_right_corner = (
            self.state is RouteState.RIGHT_CORNER_ARMED
            or self.state is RouteState.APPROACHING_RIGHT_CORNER
            or self.config.fixed_right_turn_on_line_end
        )
        if expects_right_corner and self._missing_frames < self.config.missing_before_turn:
            self.state = RouteState.APPROACHING_RIGHT_CORNER
            return self._decision(RouteIntent.STRAIGHT, "line_end_confirming")

        if expects_right_corner:
            reason = (
                "right_corner_line_end_confirmed"
                if self._right_evidence_frames >= self.config.corner_confirmation_frames
                else "fixed_route_line_end_confirmed"
            )
            return self._begin_right_turn(reason)

        self.state = RouteState.LOST
        return self._decision(RouteIntent.STOP, "unexpected_line_loss")

    def _begin_right_turn(self, reason: str) -> PlannerDecision:
        self.state = RouteState.TURNING_RIGHT
        self._turn_frames = 0
        self._reacquired_frames = 0
        return self._turn_or_stop(reason)

    def _turn_or_stop(self, reason: str) -> PlannerDecision:
        self._turn_frames += 1
        if self._turn_frames > self.config.max_turn_frames:
            self.state = RouteState.LOST
            return self._decision(RouteIntent.STOP, "right_turn_timeout")
        return self._decision(RouteIntent.TURN_RIGHT, reason)

    def _reset_follow(self) -> None:
        self.state = RouteState.FOLLOW
        self._right_evidence_frames = 0
        self._corner_absence_frames = 0
        self._missing_frames = 0
        self._turn_frames = 0
        self._reacquired_frames = 0

    def _decision(self, intent: RouteIntent, reason: str) -> PlannerDecision:
        return PlannerDecision(intent=intent, state=self.state, reason=reason)
