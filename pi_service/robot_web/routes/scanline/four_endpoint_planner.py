"""Formal state machine for the green/white I-shaped four-endpoint course.

This experiment deliberately commands no camera, HTTP endpoint, Arduino, or
card motor.  It converts camera observations into a small set of actions for a
future Pi executor.  The vehicle never drives along either crossbar: each
endpoint is served by pivoting in place at its junction.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Heading(str, Enum):
    UP = "UP"
    RIGHT = "RIGHT"
    DOWN = "DOWN"
    LEFT = "LEFT"


class Junction(str, Enum):
    TOP = "TOP"
    BOTTOM = "BOTTOM"


class Endpoint(str, Enum):
    TOP_LEFT = "1"
    TOP_RIGHT = "2"
    BOTTOM_LEFT = "3"
    BOTTOM_RIGHT = "4"


class NavigationState(str, Enum):
    FOLLOW_STEM = "FOLLOW_STEM"
    STOP_AT_JUNCTION = "STOP_AT_JUNCTION"
    PIVOT_TO_HEADING = "PIVOT_TO_HEADING"
    DEAL_CARD = "DEAL_CARD"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


class DriveAction(str, Enum):
    STOP = "STOP"
    FOLLOW_STEM = "FOLLOW_STEM"
    PIVOT_LEFT_90 = "PIVOT_LEFT_90"
    PIVOT_RIGHT_90 = "PIVOT_RIGHT_90"
    DEAL_CARD = "DEAL_CARD"


@dataclass(frozen=True)
class FourEndpointConfig:
    junction_confirm_frames: int = 2
    turn_reacquire_frames: int = 2
    start_heading: Heading = Heading.UP


@dataclass(frozen=True)
class VisionObservation:
    """Minimal evidence supplied by a later OpenCV adapter.

    ``forward_line_detected`` means the expected tape is visible near the
    bottom centre of the camera after a pivot.  The pivot must first see a
    blank/sideways phase; this prevents the incoming stem from being mistaken
    for the newly selected direction at pivot start.
    """
    junction_detected: bool
    forward_line_detected: bool
    deal_complete: bool = False


@dataclass(frozen=True)
class NavigationDecision:
    state: NavigationState
    action: DriveAction
    reason: str
    active_target: Endpoint | None
    current_junction: Junction | None
    heading: Heading
    pending_heading: Heading | None


_ENDPOINT_JUNCTION = {
    Endpoint.TOP_LEFT: Junction.TOP,
    Endpoint.TOP_RIGHT: Junction.TOP,
    Endpoint.BOTTOM_LEFT: Junction.BOTTOM,
    Endpoint.BOTTOM_RIGHT: Junction.BOTTOM,
}
_ENDPOINT_HEADING = {
    Endpoint.TOP_LEFT: Heading.LEFT,
    Endpoint.TOP_RIGHT: Heading.RIGHT,
    Endpoint.BOTTOM_LEFT: Heading.LEFT,
    Endpoint.BOTTOM_RIGHT: Heading.RIGHT,
}
_STEM_HEADING = {Junction.TOP: Heading.DOWN, Junction.BOTTOM: Heading.UP}
_LEFT_TURN = {Heading.UP: Heading.LEFT, Heading.LEFT: Heading.DOWN, Heading.DOWN: Heading.RIGHT, Heading.RIGHT: Heading.UP}
_RIGHT_TURN = {value: key for key, value in _LEFT_TURN.items()}


def _turn_action(current: Heading, desired: Heading) -> DriveAction:
    if _LEFT_TURN[current] is desired:
        return DriveAction.PIVOT_LEFT_90
    if _RIGHT_TURN[current] is desired:
        return DriveAction.PIVOT_RIGHT_90
    raise ValueError(f"{current.value} 到 {desired.value} 不是单次 90 度转向")


class FourEndpointPlanner:
    """Visit endpoints in order using only stem travel and in-place pivots."""

    def __init__(self, route: tuple[Endpoint, ...], config: FourEndpointConfig = FourEndpointConfig()) -> None:
        if not route:
            raise ValueError("路线至少需要一个端点")
        self.route = route
        self.config = config
        self.state = NavigationState.FOLLOW_STEM
        self.heading = config.start_heading
        self._route_index = 0
        self._current_junction: Junction | None = None
        self._junction_frames = 0
        self._pivot_queue: list[Heading] = []
        self._pivot_blank_seen = False
        self._pivot_reacquire_frames = 0

    @property
    def active_target(self) -> Endpoint | None:
        return self.route[self._route_index] if self._route_index < len(self.route) else None

    def _decision(self, action: DriveAction, reason: str) -> NavigationDecision:
        return NavigationDecision(
            self.state, action, reason, self.active_target,
            self._current_junction, self.heading,
            self._pivot_queue[0] if self._pivot_queue else None,
        )

    def _begin_pivots(self, headings: list[Heading]) -> NavigationDecision:
        self._pivot_queue = headings
        self._pivot_blank_seen = False
        self._pivot_reacquire_frames = 0
        self.state = NavigationState.PIVOT_TO_HEADING
        return self._decision(_turn_action(self.heading, headings[0]), "stopped_at_junction_starting_visual_90_degree_pivot")

    def _prepare_target_pivots(self) -> NavigationDecision:
        target = self.active_target
        if target is None:
            self.state = NavigationState.COMPLETE
            return self._decision(DriveAction.STOP, "all_endpoints_completed")
        target_junction, target_heading = _ENDPOINT_JUNCTION[target], _ENDPOINT_HEADING[target]
        if self._current_junction is not target_junction:
            self.state = NavigationState.FAULT
            return self._decision(DriveAction.STOP, "arrived_at_unexpected_junction")
        return self._begin_pivots([target_heading])

    def _after_deal(self) -> NavigationDecision:
        self._route_index += 1
        next_target = self.active_target
        if next_target is None:
            self.state = NavigationState.COMPLETE
            return self._decision(DriveAction.STOP, "all_endpoints_completed")
        next_junction = _ENDPOINT_JUNCTION[next_target]
        if next_junction is self._current_junction:
            # No crossbar travel.  Return through the stem-facing direction,
            # then make a second visual 90-degree pivot to the other endpoint.
            return self._begin_pivots([_STEM_HEADING[next_junction], _ENDPOINT_HEADING[next_target]])
        return self._begin_pivots([_STEM_HEADING[self._current_junction]])

    def step(self, observation: VisionObservation) -> NavigationDecision:
        if self.state is NavigationState.FOLLOW_STEM:
            target = self.active_target
            if target is None:
                self.state = NavigationState.COMPLETE
                return self._decision(DriveAction.STOP, "all_endpoints_completed")
            self._junction_frames = self._junction_frames + 1 if observation.junction_detected else 0
            if self._junction_frames < self.config.junction_confirm_frames:
                return self._decision(DriveAction.FOLLOW_STEM, "following_stem_toward_expected_junction")
            self._current_junction = _ENDPOINT_JUNCTION[target]
            self.state = NavigationState.STOP_AT_JUNCTION
            return self._decision(DriveAction.STOP, "expected_junction_confirmed_stop_before_pivot")

        if self.state is NavigationState.STOP_AT_JUNCTION:
            return self._prepare_target_pivots()

        if self.state is NavigationState.PIVOT_TO_HEADING:
            desired = self._pivot_queue[0]
            if not observation.forward_line_detected:
                self._pivot_blank_seen = True
                return self._decision(_turn_action(self.heading, desired), "pivoting_waiting_for_new_direction_line")
            if not self._pivot_blank_seen:
                return self._decision(_turn_action(self.heading, desired), "pivoting_waiting_for_blank_phase")
            self._pivot_reacquire_frames += 1
            if self._pivot_reacquire_frames < self.config.turn_reacquire_frames:
                return self._decision(_turn_action(self.heading, desired), "pivoting_confirming_new_direction_line")
            self.heading = desired
            self._pivot_queue.pop(0)
            self._pivot_blank_seen = False
            self._pivot_reacquire_frames = 0
            if self._pivot_queue:
                return self._decision(_turn_action(self.heading, self._pivot_queue[0]), "first_90_degree_pivot_confirmed_starting_second")
            if self.heading is _ENDPOINT_HEADING[self.active_target]:
                self.state = NavigationState.DEAL_CARD
                return self._decision(DriveAction.STOP, "endpoint_facing_heading_confirmed_ready_to_deal")
            self.state = NavigationState.FOLLOW_STEM
            self._junction_frames = 0
            return self._decision(DriveAction.FOLLOW_STEM, "stem_facing_heading_confirmed_driving_to_next_junction")

        if self.state is NavigationState.DEAL_CARD:
            if not observation.deal_complete:
                return self._decision(DriveAction.DEAL_CARD, "holding_position_while_dealing")
            return self._after_deal()

        if self.state is NavigationState.COMPLETE:
            return self._decision(DriveAction.STOP, "all_endpoints_completed")
        return self._decision(DriveAction.STOP, "planner_fault")
