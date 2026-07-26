"""PC-side two-red-band state machine for the fixed I-course.

The physical band farther from the T is encountered first.  While both layers
are visible it is still an approach warning.  When that first band leaves the
frame, the remaining layer is the turn band; fish-eye skew is handled by the
analyzer's fragment-to-layer grouping rather than comparing two exact Y values.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class RedLayerView:
    y: int
    bottom_y: int
    span: int


@dataclass(frozen=True)
class RedBandConfig:
    minimum_span_ratio: float = .18
    brake_y_ratio: float = .50
    pivot_y_ratio: float = .84
    # Arm overshoot recovery before the pre-authorized 70% pivot threshold;
    # otherwise a dropped frame could never produce REVERSE_REQUEST.
    exit_arm_y_ratio: float = .60
    preauthorized_brake_y_ratio: float = .35
    preauthorized_pivot_y_ratio: float = .70


@dataclass(frozen=True)
class RedBandDecision:
    event: str
    phase: str
    layer_count: int
    turn_y: int | None
    turn_bottom_y: int | None


class TwoRedBandPlanner:
    """Translate visible red layers into safe high-level adaptor events.

    The output is deliberately idempotent.  The PC may resend it for every
    frame; the Pi performs brake/pivot/reverse only when the event type changes.
    """

    def __init__(self, config: RedBandConfig = RedBandConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._two_layers_seen = False
        self._first_band_confirmed = False
        self._turn_band_seen = False
        self._brake_sent = False
        self._turn_bottom_armed = False
        self._pivot_sent = False

    def update(self, layers: Iterable[object], frame_width: int, frame_height: int) -> RedBandDecision:
        valid = [
            RedLayerView(int(layer.y), int(layer.bottom_y), int(layer.span))
            for layer in layers
            if int(layer.span) >= frame_width * self.config.minimum_span_ratio
        ]
        # Red tape can be split around the white stem; that grouping occurred
        # before this point.  Any remaining third blob is noise: use the two
        # widest physical layers, then restore their image-Y order.
        if len(valid) > 2:
            valid = sorted(valid, key=lambda layer: layer.span, reverse=True)[:2]
        valid.sort(key=lambda layer: layer.y)
        if len(valid) >= 2:
            self._two_layers_seen = True
            self._first_band_confirmed = True
            return RedBandDecision("SLOW_DOWN", "TWO_LAYERS_APPROACH", len(valid), None, None)
        if len(valid) == 1:
            layer = valid[0]
            if not self._two_layers_seen:
                self._first_band_confirmed = True
                return RedBandDecision("SLOW_DOWN", "FIRST_BAND_APPROACH", 1, layer.y, layer.bottom_y)
            self._turn_band_seen = True
            self._turn_bottom_armed = self._turn_bottom_armed or layer.bottom_y >= frame_height * self.config.exit_arm_y_ratio
            brake_ratio = self.config.preauthorized_brake_y_ratio if self._first_band_confirmed else self.config.brake_y_ratio
            pivot_ratio = self.config.preauthorized_pivot_y_ratio if self._first_band_confirmed else self.config.pivot_y_ratio
            if not self._pivot_sent and layer.bottom_y >= frame_height * pivot_ratio:
                self._pivot_sent = True
                return RedBandDecision("PIVOT_REQUEST", "TURN_BAND_BOTTOM", 1, layer.y, layer.bottom_y)
            if not self._brake_sent and layer.y >= frame_height * brake_ratio:
                self._brake_sent = True
                return RedBandDecision("BRAKE_NOW", "TURN_BAND_MID", 1, layer.y, layer.bottom_y)
            return RedBandDecision("SLOW_DOWN", "TURN_BAND_CREEP", 1, layer.y, layer.bottom_y)
        if self._turn_band_seen and not self._pivot_sent and self._turn_bottom_armed:
            return RedBandDecision("REVERSE_REQUEST", "TURN_BAND_EXITED_BOTTOM", 0, None, None)
        return RedBandDecision("CLEAR_ARM", "NO_RED", 0, None, None)
