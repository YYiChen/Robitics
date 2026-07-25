"""Pi-local red-band detector for the fixed green-course experiment.

This module intentionally has no skeleton, white-tape, or green-field work.
It receives the already decoded Pi BGR frame, restricts red detection to the
known central course ROI, and emits the same high-level events as the PC
planner.  The PC remains free to run its full analysis for diagnostics.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from red_band_planner import RedBandConfig, RedBandDecision, TwoRedBandPlanner


@dataclass(frozen=True)
class PiFastRedConfig:
    red_channel_min: int = 120
    red_excess_min: int = 44
    roi_top_ratio: float = .20
    roi_side_ratio: float = .25
    min_component_area: int = 12
    group_y_tolerance_ratio: float = .10
    brake_y_ratio: float = .50
    pivot_bottom_y_ratio: float = .84
    exit_arm_y_ratio: float = .60


@dataclass(frozen=True)
class PiFastRedLayer:
    y: int
    bottom_y: int
    span: int
    fragment_count: int


class PiFastRedBandPlanner:
    """ROI-gated red layer detection plus the fixed two-band state machine."""

    def __init__(self, config: PiFastRedConfig = PiFastRedConfig()) -> None:
        self.config = config
        # Do not require a horizontal-span threshold here: the fixed ROI and
        # layer grouping are the intentional scene-specific discriminator.
        self._planner = TwoRedBandPlanner(RedBandConfig(
            minimum_span_ratio=0.0,
            brake_y_ratio=config.brake_y_ratio,
            pivot_y_ratio=config.pivot_bottom_y_ratio,
            exit_arm_y_ratio=config.exit_arm_y_ratio,
            preauthorized_brake_y_ratio=config.brake_y_ratio,
            preauthorized_pivot_y_ratio=config.pivot_bottom_y_ratio,
        ))

    def reset(self) -> None:
        self._planner.reset()

    def detect_layers(self, frame: np.ndarray) -> tuple[PiFastRedLayer, ...]:
        height, width = frame.shape[:2]
        blue, green, red_channel = cv2.split(frame)
        red_excess = red_channel.astype(np.int16) - np.maximum(green, blue).astype(np.int16)
        red = np.where(
            (red_channel >= self.config.red_channel_min) & (red_excess >= self.config.red_excess_min),
            255,
            0,
        ).astype(np.uint8)
        top = max(0, min(height, int(round(height * self.config.roi_top_ratio))))
        side = max(0, min(width // 2, int(round(width * self.config.roi_side_ratio))))
        red[:top, :] = 0
        red[:, :side] = 0
        red[:, width - side:] = 0
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        count, _labels, stats, centroids = cv2.connectedComponentsWithStats(red, connectivity=8)
        fragments: list[tuple[int, int, int, int, int, float]] = []
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            if area < self.config.min_component_area:
                continue
            fragments.append((x, y, component_width, component_height, area, float(centroids[label][1])))
        fragments.sort(key=lambda item: item[5])
        tolerance = height * self.config.group_y_tolerance_ratio
        groups: list[list[tuple[int, int, int, int, int, float]]] = []
        for fragment in fragments:
            if not groups or fragment[5] - groups[-1][-1][5] > tolerance:
                groups.append([fragment])
            else:
                groups[-1].append(fragment)
        layers: list[PiFastRedLayer] = []
        for group in groups:
            area = sum(item[4] for item in group)
            left = min(item[0] for item in group)
            right = max(item[0] + item[2] - 1 for item in group)
            marker_y = int(round(sum(item[4] * item[5] for item in group) / area))
            bottom_y = max(item[1] + item[3] - 1 for item in group)
            layers.append(PiFastRedLayer(marker_y, bottom_y, right - left + 1, len(group)))
        return tuple(sorted(layers, key=lambda layer: layer.y))

    def step(self, frame: np.ndarray) -> tuple[RedBandDecision, tuple[PiFastRedLayer, ...]]:
        layers = self.detect_layers(frame)
        decision = self._planner.update(layers, frame.shape[1], frame.shape[0])
        return decision, layers
