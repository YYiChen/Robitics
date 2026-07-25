"""Node-level stop decisions for a fixed course, independent from motors."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class NodeState(str, Enum):
    FOLLOW = "FOLLOW"
    APPROACH_NODE = "APPROACH_NODE"
    HOLD_NODE = "HOLD_NODE"
    WAIT_RESUME = "WAIT_RESUME"


@dataclass(frozen=True)
class NodePlannerConfig:
    confirm_frames: int = 2
    clear_frames: int = 12
    stop_y_ratio: float = 0.72
    hold_seconds: float = 2.0
    nodes_per_lap: int = 4
    auto_resume: bool = True

    def __post_init__(self) -> None:
        if self.confirm_frames < 1 or self.clear_frames < 1 or self.nodes_per_lap < 1:
            raise ValueError("frame and node counts must be positive")
        if not 0 < self.stop_y_ratio < 1 or self.hold_seconds < 0:
            raise ValueError("node stop geometry is invalid")


@dataclass(frozen=True)
class NodeDecision:
    state: NodeState
    should_stop: bool
    reason: str
    next_node: int
    completed_node: int | None
    lap_count: int


class NodePlanner:
    """Turn confirmed transverse markers into ordered, debounced stop nodes."""

    def __init__(self, config: NodePlannerConfig | None = None) -> None:
        self.config = config or NodePlannerConfig()
        self._state = NodeState.FOLLOW
        self._present_frames = 0
        self._clear_frames = 0
        self._await_clear = False
        self._hold_until: float | None = None
        self._completed_total = 0
        self._active_node: int | None = None
        self._resume_requested = False

    @property
    def next_node(self) -> int:
        return self._completed_total % self.config.nodes_per_lap + 1

    def resume(self) -> None:
        """Future card-dealing code may call this after a node is safe to leave."""
        self._resume_requested = True

    def _enter_hold(self, now: float) -> int:
        completed = self._active_node or self.next_node
        self._completed_total += 1
        self._hold_until = now + self.config.hold_seconds
        self._state = NodeState.HOLD_NODE
        self._await_clear = True
        self._clear_frames = 0
        return completed

    def step(self, *, marker_detected: bool, marker_y_ratio: float | None, now: float) -> NodeDecision:
        completed: int | None = None
        if self._await_clear:
            self._clear_frames = self._clear_frames + 1 if not marker_detected else 0
            if self._clear_frames >= self.config.clear_frames:
                self._await_clear = False
                self._clear_frames = 0

        if self._state is NodeState.FOLLOW:
            if marker_detected and not self._await_clear:
                self._present_frames += 1
                if self._present_frames >= self.config.confirm_frames:
                    self._active_node = self.next_node
                    if marker_y_ratio is not None and marker_y_ratio >= self.config.stop_y_ratio:
                        completed = self._enter_hold(now)
                    else:
                        self._state = NodeState.APPROACH_NODE
            elif not marker_detected:
                self._present_frames = 0

        elif self._state is NodeState.APPROACH_NODE:
            if marker_detected and marker_y_ratio is not None and marker_y_ratio >= self.config.stop_y_ratio:
                completed = self._enter_hold(now)
            elif not marker_detected:
                # A noisy candidate must be reconfirmed before it can stop the route.
                self._state = NodeState.FOLLOW
                self._present_frames = 0
                self._active_node = None

        elif self._state is NodeState.HOLD_NODE:
            if self.config.auto_resume and self._hold_until is not None and now >= self._hold_until:
                self._state = NodeState.FOLLOW
                self._present_frames = 0
                self._active_node = None
            elif not self.config.auto_resume:
                self._state = NodeState.WAIT_RESUME

        elif self._state is NodeState.WAIT_RESUME and self._resume_requested:
            self._resume_requested = False
            self._state = NodeState.FOLLOW
            self._present_frames = 0
            self._active_node = None

        stop = self._state in {NodeState.HOLD_NODE, NodeState.WAIT_RESUME}
        reason = {
            NodeState.FOLLOW: "following_route",
            NodeState.APPROACH_NODE: "confirmed_marker_waiting_for_stop_position",
            NodeState.HOLD_NODE: "node_hold_timer",
            NodeState.WAIT_RESUME: "node_waiting_for_external_resume",
        }[self._state]
        return NodeDecision(
            state=self._state,
            should_stop=stop,
            reason=reason,
            next_node=self.next_node,
            completed_node=completed,
            lap_count=self._completed_total // self.config.nodes_per_lap,
        )
