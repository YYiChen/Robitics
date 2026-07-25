"""Straight white line and red terminal-band logic.

This deliberately contains no skeleton, T-junction, transverse-white-bar,
two-red-band, PC-offload, or 180-degree turnaround behaviour.  The present
course has one white stem and one red terminal band at each end.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np


@dataclass(frozen=True)
class EndLineConfig:
    red_channel_min: int = 120
    red_excess_min: int = 44
    red_roi_top_ratio: float = .18
    red_roi_side_ratio: float = .04
    red_min_component_area: int = 100
    red_min_span_ratio: float = .25
    red_confirm_frames: int = 2
    red_stop_bottom_ratio: float = .80
    line_lost_confirm_frames: int = 3


@dataclass(frozen=True)
class RedEndBandObservation:
    detected: bool
    center_x: int | None = None
    y: int | None = None
    bottom_y: int | None = None
    span: int | None = None
    area: int = 0


class EndLineState(str, Enum):
    FOLLOW_LINE = "FOLLOW_LINE"
    RED_BAND_APPROACH = "RED_BAND_APPROACH"
    STOPPED_RED_BAND = "STOPPED_RED_BAND"
    STOPPED_RED_LINE_LOST = "STOPPED_RED_LINE_LOST"
    STOPPED_UNSAFE_LINE_LOST = "STOPPED_UNSAFE_LINE_LOST"


@dataclass(frozen=True)
class EndLineDecision:
    state: EndLineState
    stop: bool
    reason: str


class RedEndBandDetector:
    """Detect the single wide red terminal band without treating it as tape."""

    def __init__(self, config: EndLineConfig = EndLineConfig()) -> None:
        self.config = config

    def detect(self, frame: np.ndarray) -> RedEndBandObservation:
        height, width = frame.shape[:2]
        blue, green, red_channel = cv2.split(frame)
        red_excess = red_channel.astype(np.int16) - np.maximum(blue, green).astype(np.int16)
        mask = np.where(
            (red_channel >= self.config.red_channel_min) & (red_excess >= self.config.red_excess_min), 255, 0
        ).astype(np.uint8)
        top = int(height * self.config.red_roi_top_ratio)
        side = int(width * self.config.red_roi_side_ratio)
        mask[:top, :] = 0
        mask[:, :side] = 0
        mask[:, width - side:] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = []
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            if area < self.config.red_min_component_area or component_width < width * self.config.red_min_span_ratio:
                continue
            candidates.append((area, x, y, component_width, component_height, centroids[label]))
        if not candidates:
            return RedEndBandObservation(False)
        area, x, y, component_width, component_height, centroid = max(candidates, key=lambda item: item[0])
        return RedEndBandObservation(True, int(round(centroid[0])), int(round(centroid[1])), y + component_height - 1, component_width, area)


class EndLineStopPlanner:
    """Stop at a confirmed red terminal; stop safely on an unexpected line loss."""

    def __init__(self, config: EndLineConfig = EndLineConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._red_frames = 0
        self._line_lost_frames = 0
        self._red_confirmed = False
        self._stopped: EndLineDecision | None = None

    def step(self, *, line_valid: bool, red_band: RedEndBandObservation, frame_height: int) -> EndLineDecision:
        if self._stopped is not None:
            return self._stopped
        if red_band.detected:
            self._red_frames += 1
            self._red_confirmed = self._red_confirmed or self._red_frames >= self.config.red_confirm_frames
        else:
            self._red_frames = 0
        self._line_lost_frames = 0 if line_valid else self._line_lost_frames + 1
        if self._red_confirmed and red_band.detected and (red_band.bottom_y or 0) >= frame_height * self.config.red_stop_bottom_ratio:
            self._stopped = EndLineDecision(EndLineState.STOPPED_RED_BAND, True, "confirmed_red_terminal_reached_stop_zone")
        elif self._red_confirmed and self._line_lost_frames >= self.config.line_lost_confirm_frames:
            self._stopped = EndLineDecision(EndLineState.STOPPED_RED_LINE_LOST, True, "confirmed_red_terminal_then_white_line_lost")
        elif not self._red_confirmed and self._line_lost_frames >= self.config.line_lost_confirm_frames:
            self._stopped = EndLineDecision(EndLineState.STOPPED_UNSAFE_LINE_LOST, True, "white_line_lost_without_confirmed_red_terminal")
        elif self._red_confirmed:
            return EndLineDecision(EndLineState.RED_BAND_APPROACH, False, "confirmed_red_terminal_approach")
        else:
            return EndLineDecision(EndLineState.FOLLOW_LINE, False, "following_single_white_line")
        return self._stopped
