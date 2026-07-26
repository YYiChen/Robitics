"""Formal debounce of same-colour X/T tape markers into route/lap counts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MarkerState(str, Enum):
    ARMED = "ARMED"
    CONFIRMING = "CONFIRMING"
    COOLDOWN = "COOLDOWN"


@dataclass(frozen=True)
class MarkerCounterConfig:
    """Parameters for one physical marker crossing, not motor control."""

    confirm_frames: int = 2
    clear_frames: int = 4
    markers_per_lap: int = 4
    # If a still-visible candidate suddenly returns higher in the image, it is
    # a new physical marker, not the tail of the previous one.
    rearm_y_drop_ratio: float = 0.18


@dataclass(frozen=True)
class MarkerUpdate:
    detected: bool
    event: bool
    total_markers: int
    marker_in_lap: int
    lap_count: int
    state: MarkerState


class MarkerCounter:
    """Count a marker once, then require it to leave the camera before rearming."""

    def __init__(self, config: MarkerCounterConfig | None = None) -> None:
        self.config = config or MarkerCounterConfig()
        if self.config.confirm_frames < 1 or self.config.clear_frames < 1 or self.config.markers_per_lap < 1:
            raise ValueError("marker counter settings must be positive")
        if not 0 < self.config.rearm_y_drop_ratio < 1:
            raise ValueError("rearm_y_drop_ratio must be in (0, 1)")
        self._state = MarkerState.ARMED
        self._present_frames = 0
        self._clear_frames = 0
        self._total = 0
        self._last_event_y_ratio: float | None = None

    def update(self, detected: bool, marker_y_ratio: float | None = None) -> MarkerUpdate:
        event = False
        if self._state is MarkerState.ARMED:
            if detected:
                self._present_frames = 1
                self._state = MarkerState.CONFIRMING
        elif self._state is MarkerState.CONFIRMING:
            if detected:
                self._present_frames += 1
                if self._present_frames >= self.config.confirm_frames:
                    self._total += 1
                    event = True
                    self._clear_frames = 0
                    self._last_event_y_ratio = marker_y_ratio
                    self._state = MarkerState.COOLDOWN
            else:
                self._present_frames = 0
                self._state = MarkerState.ARMED
        else:  # COOLDOWN: the same X can be visible in many consecutive frames.
            reappeared_higher = (
                detected
                and marker_y_ratio is not None
                and self._last_event_y_ratio is not None
                and marker_y_ratio <= self._last_event_y_ratio - self.config.rearm_y_drop_ratio
            )
            if reappeared_higher:
                # Detector leakage can keep the old marker true for too long.
                # A new marker first appears high in the image, so safely rearm
                # and require the normal confirmation frames again.
                self._present_frames = 0
                self._clear_frames = 0
                self._last_event_y_ratio = None
                self._state = MarkerState.ARMED
            elif detected:
                self._clear_frames = 0
            else:
                self._clear_frames += 1
                if self._clear_frames >= self.config.clear_frames:
                    self._present_frames = 0
                    self._state = MarkerState.ARMED

        return MarkerUpdate(
            detected=detected,
            event=event,
            total_markers=self._total,
            marker_in_lap=self._total % self.config.markers_per_lap,
            lap_count=self._total // self.config.markers_per_lap,
            state=self._state,
        )
