"""Pure-Python 90-degree-turn decision logic for the robot's action protocol.

This is intentionally separate from robot_web: it verifies the decision layer
without opening the camera or sending commands to the Arduino.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    FOLLOW = "FOLLOW"
    SEARCH_LEFT = "SEARCH_LEFT"
    SEARCH_RIGHT = "SEARCH_RIGHT"
    RECOVER = "RECOVER"
    LOST = "LOST"


@dataclass(frozen=True)
class TrackerConfig:
    centre_deadband_px: int = 28
    curve_threshold_px: int = 85
    missing_before_search: int = 2
    reacquire_frames: int = 2
    max_search_frames: int = 30


class RightAngleTracker:
    """Convert a detected line centre (or None) to existing controller actions."""

    def __init__(self, image_centre_x: int, config: TrackerConfig = TrackerConfig()) -> None:
        self.image_centre_x = image_centre_x
        self.config = config
        self.state = State.FOLLOW
        self._last_error = 0
        self._missing_frames = 0
        self._search_frames = 0
        self._reacquired_frames = 0

    def step(self, line_centre_x: int | None) -> str:
        """Return F/FL/FR/PL/PR/STOP for one camera frame.

        At a square corner the old line disappears from the lower image region.
        The remembered side selects a slow pivot until the perpendicular line is
        seen again.  This is what simple proportional following lacks.
        """
        if line_centre_x is None:
            return self._on_missing_line()

        error = line_centre_x - self.image_centre_x
        self._last_error = error if error else self._last_error
        self._missing_frames = 0
        self._search_frames = 0

        if self.state in (State.SEARCH_LEFT, State.SEARCH_RIGHT, State.LOST):
            self.state = State.RECOVER
            self._reacquired_frames = 1
            return "STOP"  # brake before committing to the newly found line
        if self.state is State.RECOVER:
            self._reacquired_frames += 1
            if self._reacquired_frames < self.config.reacquire_frames:
                return "STOP"
            self.state = State.FOLLOW

        if abs(error) <= self.config.centre_deadband_px:
            return "F"
        return "FL" if error < 0 else "FR"

    def _on_missing_line(self) -> str:
        self._missing_frames += 1
        if self.state is State.FOLLOW and self._missing_frames >= self.config.missing_before_search:
            self.state = State.SEARCH_LEFT if self._last_error <= 0 else State.SEARCH_RIGHT
            self._search_frames = 0
        if self.state in (State.SEARCH_LEFT, State.SEARCH_RIGHT):
            self._search_frames += 1
            if self._search_frames > self.config.max_search_frames:
                self.state = State.LOST
                return "STOP"
            return "PL" if self.state is State.SEARCH_LEFT else "PR"
        return "STOP"
