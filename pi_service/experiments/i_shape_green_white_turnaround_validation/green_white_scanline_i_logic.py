"""Green-floor / white-tape mask for the proven I-shape scanline state machine."""
from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import sys

import cv2
import numpy as np


BLACK_LINE_EXPERIMENT = Path(__file__).resolve().parents[1] / "i_shape_scanline_turnaround_validation"
if str(BLACK_LINE_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(BLACK_LINE_EXPERIMENT))

from scanline_i_logic import HybridScanlineAnalyzer, HybridScanlineConfig  # noqa: E402


@dataclass(frozen=True)
class GreenWhiteScanlineConfig(HybridScanlineConfig):
    """HSV thresholds verified by the fixed green-course detector config."""
    # Keep the original permissive white threshold.  At oblique fisheye
    # angles the tape picks up green/blue reflection and is no longer nearly
    # neutral or bright enough for the stricter experimental values.
    white_saturation_max: int = 82
    white_value_min: int = 168
    green_hue_min: int = 32
    green_hue_max: int = 96
    green_saturation_min: int = 45
    green_value_min: int = 28
    minimum_green_roi_ratio: float = 0.18
    roi_top_ratio: float = 0.38
    green_neighbour_kernel: int = 31
    # A fisheye view makes a straight physical tape sweep sideways between the
    # three near scan rows.  Preserve the original permissive recognition
    # behaviour for this green/white course instead of declaring that normal
    # perspective change to be a lost route.
    maximum_center_spread_ratio: float = 0.18
    near_track_max_width_ratio: float = 0.22
    near_track_centre_tolerance_ratio: float = 0.24
    minimum_near_track_rows: int = 2
    red_hue_low_max: int = 12
    red_hue_high_min: int = 165
    red_saturation_min: int = 85
    red_value_min: int = 70
    red_min_component_area_ratio: float = 0.00015
    red_min_span_ratio: float = 0.18
    red_group_y_tolerance_ratio: float = 0.06
    red_centre_tolerance_ratio: float = 0.20


class GreenWhiteHybridScanlineAnalyzer(HybridScanlineAnalyzer):
    """Reuse all I-turn geometry; replace only black-tape Otsu segmentation."""
    def __init__(self, config: GreenWhiteScanlineConfig = GreenWhiteScanlineConfig()) -> None:
        super().__init__(config)
        self.config = config

    @staticmethod
    def _odd(value: int) -> int:
        return value if value % 2 else value + 1

    def _make_mask(self, frame: np.ndarray, blur_kernel: int, morphology_kernel: int) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        config = self.config
        white = cv2.inRange(
            hsv,
            np.array((0, 0, config.white_value_min), dtype=np.uint8),
            np.array((180, config.white_saturation_max, 255), dtype=np.uint8),
        )
        green = cv2.inRange(
            hsv,
            np.array((config.green_hue_min, config.green_saturation_min, config.green_value_min), dtype=np.uint8),
            np.array((config.green_hue_max, 255, 255), dtype=np.uint8),
        )
        height = frame.shape[0]
        roi_start = min(height - 1, max(0, int(round(height * config.roi_top_ratio))))
        green_roi_ratio = float(np.count_nonzero(green[roi_start:])) / max(1, green[roi_start:].size)
        if green_roi_ratio < config.minimum_green_roi_ratio:
            return np.zeros_like(white)
        # Restore the proven permissive mask: tape only needs to touch the
        # green course.  Oblique views and the T intersection do not preserve
        # a reliable green pixel on both sides of every tape pixel.  The
        # stricter route-component selector below remains responsible for
        # rejecting a broad pale floor patch.
        neighbour_size = self._odd(max(3, config.green_neighbour_kernel))
        neighbour = cv2.dilate(green, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (neighbour_size, neighbour_size)))
        mask = cv2.bitwise_and(white, neighbour)
        cleanup_size = self._odd(max(3, morphology_kernel))
        cleanup = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cleanup_size, cleanup_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cleanup)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cleanup)

    def _select_route_component(self, mask: np.ndarray) -> np.ndarray:
        """Choose only a near, centred narrow-tape component as the route.

        The parent scorer intentionally favours a large near component.  On a
        green-mat boundary that makes a large pale floor patch beat the tape.
        A valid driving route must instead look like a narrow strip on at
        least two bottom scan rows; the transverse bar is still retained in
        the same connected component for endpoint evidence.
        """
        constrained = cv2.bitwise_and(mask, self._route_corridor_mask(mask.shape))
        connection_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.route_connection_kernel, self.config.route_connection_kernel),
        )
        constrained = cv2.morphologyEx(constrained, cv2.MORPH_CLOSE, connection_kernel)
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(constrained, connectivity=8)
        height, width = constrained.shape
        near_start = int(round(height * (1.0 - self.config.route_anchor_near_ratio)))
        near_labels = labels[near_start:, :]
        track_ys = tuple(min(height - 1, max(0, int(round(height * ratio)))) for ratio in self.config.track_rows)
        maximum_width = width * self.config.near_track_max_width_ratio
        centre_tolerance = width * self.config.near_track_centre_tolerance_ratio
        best_label, best_score = 0, float("-inf")
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            if component_height / max(1, height) < self.config.route_minimum_vertical_coverage:
                continue
            near_count = int(np.count_nonzero(near_labels == label))
            if near_count == 0:
                continue
            narrow_rows = 0
            for row_y in track_ys:
                xs = np.flatnonzero(labels[row_y] == label)
                if xs.size == 0:
                    continue
                gaps = np.flatnonzero(np.diff(xs) > 1)
                starts = np.r_[0, gaps + 1]
                ends = np.r_[gaps, xs.size - 1]
                for start, end in zip(starts, ends):
                    left, right = int(xs[start]), int(xs[end])
                    run_width = right - left + 1
                    if run_width <= maximum_width and abs((left + right) / 2.0 - width / 2.0) <= centre_tolerance:
                        narrow_rows += 1
                        break
            if narrow_rows < self.config.minimum_near_track_rows:
                continue
            centre_distance = abs((x + component_width / 2.0) - width / 2.0) / max(1, width)
            score = narrow_rows * width * height + near_count * 2.0 + area * .05 - centre_distance * area * .10
            if score > best_score:
                best_label, best_score = label, score
        if best_label == 0:
            return np.zeros_like(constrained)
        return np.where(labels == best_label, 255, 0).astype(np.uint8)

    def _detect_red_band_marker(self, frame: np.ndarray, route_center_x: float | None) -> tuple[bool, int | None, int | None]:
        """Find the two red fragments flanking the incoming white stem."""
        height, width = frame.shape[:2]
        config = self.config
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_low = cv2.inRange(hsv, np.array((0, config.red_saturation_min, config.red_value_min), dtype=np.uint8), np.array((config.red_hue_low_max, 255, 255), dtype=np.uint8))
        red_high = cv2.inRange(hsv, np.array((config.red_hue_high_min, config.red_saturation_min, config.red_value_min), dtype=np.uint8), np.array((180, 255, 255), dtype=np.uint8))
        green = cv2.inRange(hsv, np.array((config.green_hue_min, config.green_saturation_min, config.green_value_min), dtype=np.uint8), np.array((config.green_hue_max, 255, 255), dtype=np.uint8))
        neighbour_size = self._odd(max(3, config.green_neighbour_kernel))
        green_neighbour = cv2.dilate(green, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (neighbour_size, neighbour_size)))
        cleanup = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red = cv2.morphologyEx(cv2.bitwise_and(cv2.bitwise_or(red_low, red_high), green_neighbour), cv2.MORPH_OPEN, cleanup)
        count, _labels, stats, centroids = cv2.connectedComponentsWithStats(red, connectivity=8)
        minimum_area = max(12, int(round(width * height * config.red_min_component_area_ratio)))
        fragments: list[tuple[int, int, int, int, int, float, float]] = []
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            if area < minimum_area or component_width < 4 or component_height < 3:
                continue
            cx, cy = centroids[label]
            if height * .15 <= cy <= height * .92:
                fragments.append((x, y, component_width, component_height, area, float(cx), float(cy)))
        expected_x = width / 2.0 if route_center_x is None else route_center_x
        best: tuple[float, int, int] | None = None
        for _x, _y, _w, _h, _area, _cx, seed_y in fragments:
            group = [item for item in fragments if abs(item[6] - seed_y) <= height * config.red_group_y_tolerance_ratio]
            if len(group) < 2:
                continue
            left = min(item[0] for item in group)
            right = max(item[0] + item[2] - 1 for item in group)
            span = right - left + 1
            centre = (left + right) / 2.0
            if span < width * config.red_min_span_ratio or abs(centre - expected_x) > width * config.red_centre_tolerance_ratio:
                continue
            total_area = sum(item[4] for item in group)
            marker_y = int(round(sum(item[4] * item[6] for item in group) / total_area))
            score = span + min(total_area / max(1, height), width * .20)
            if best is None or score > best[0]:
                best = (score, marker_y, span)
        return (best is not None, None if best is None else best[1], None if best is None else best[2])

    def analyze(self, frame: np.ndarray):
        result = super().analyze(frame)
        evidence = result.evidence
        detected, marker_y, marker_span = self._detect_red_band_marker(frame, evidence.line_center_x)
        if not detected or marker_y is None:
            return result
        # Red only pre-authorizes the white T. White endpoint confirmation and
        # stem loss are still required before braking or pivoting.
        junction_y = max(value for value in (evidence.junction_y, marker_y) if value is not None)
        red_evidence = replace(evidence, junction_detected=True, junction_y=junction_y, red_marker_detected=True, red_marker_y=marker_y, red_marker_span=marker_span)
        return result.__class__(red_evidence, result.component_mask)
