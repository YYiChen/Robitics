"""Approximate ground-plane coordinates for a camera-mounted line detector.

The track corridor is a trapezoid in camera pixels but approximately a
rectangle on the floor.  A homography maps that trapezoid to a normalized
bird's-eye plane: x=-1 is left, x=0 is the vehicle centre, x=+1 is right.
This is deliberately a *rough* calibration; it stabilizes control before a
full chessboard camera calibration is needed.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class GroundPlaneConfig:
    history_size: int = 5
    smoothing_alpha: float = 0.35
    max_offset_step: float = 0.08
    # y=0 is far ahead and y=1 is beside the vehicle in the normalized plane.
    # This point is deliberately ahead of the vehicle, not the closest tape.
    lookahead_y: float = 0.55


class GroundPlaneLineFilter:
    """Project line points to ground coordinates and suppress frame jitter."""

    def __init__(self, config: GroundPlaneConfig = GroundPlaneConfig()) -> None:
        self.config = config
        self._offset_history: deque[float] = deque(maxlen=config.history_size)
        self._heading_history: deque[float] = deque(maxlen=config.history_size)
        self._offset: float | None = None
        self._heading: float | None = None

    @staticmethod
    def project(points_px: Iterable[tuple[int, int]], corridor_polygon: np.ndarray) -> np.ndarray:
        """Return normalized ground points with x in [-1, 1], y in [0, 1]."""
        source = np.asarray(corridor_polygon, dtype=np.float32)
        if source.shape != (4, 2):
            raise ValueError("corridor polygon must contain four points")
        points = np.asarray(tuple(points_px), dtype=np.float32)
        if not len(points):
            return np.empty((0, 2), dtype=np.float32)
        destination = np.asarray(
            [(-1.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-1.0, 1.0)],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(source, destination)
        return cv2.perspectiveTransform(points.reshape(-1, 1, 2), matrix).reshape(-1, 2)

    def update(
        self,
        *,
        line_lost: bool,
        points_px: Iterable[tuple[int, int]],
        corridor_polygon: np.ndarray,
    ) -> tuple[float | None, float | None]:
        """Return filtered ground offset and heading, or ``None`` if unusable."""
        if line_lost:
            return None, None
        ground = self.project(points_px, corridor_polygon)
        if len(ground) < 2:
            return None, None

        # Camera-band order is not a contract, so sort explicitly: small y is
        # farther away and large y is closer to the vehicle.
        ground = ground[np.argsort(ground[:, 1])]
        # Fit the centreline and evaluate it at a configurable point ahead of
        # the vehicle.  This is the visual equivalent of a short forward
        # prediction, so a diagonal new edge is followed instead of being
        # treated as an instruction to keep pivoting in place.
        slope, intercept = np.polyfit(ground[:, 1], ground[:, 0], deg=1)
        raw_offset = float(np.clip(slope * self.config.lookahead_y + intercept, -1.0, 1.0))
        raw_heading = float(ground[0, 0] - ground[-1, 0])
        self._offset_history.append(raw_offset)
        self._heading_history.append(raw_heading)
        median_offset = float(np.median(self._offset_history))
        median_heading = float(np.median(self._heading_history))

        if self._offset is None:
            self._offset = median_offset
            self._heading = median_heading
        else:
            step = float(
                np.clip(
                    median_offset - self._offset,
                    -self.config.max_offset_step,
                    self.config.max_offset_step,
                )
            )
            alpha = self.config.smoothing_alpha
            self._offset += alpha * step
            self._heading = alpha * median_heading + (1.0 - alpha) * (self._heading or 0.0)
        return self._offset, self._heading
