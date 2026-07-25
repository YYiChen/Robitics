"""I-shape route evidence from scanlines, with optional skeleton-augmented hybrid."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import statistics

import cv2
import numpy as np


@dataclass(frozen=True)
class ScanlineConfig:
    # The terminal bar can fill the middle of the picture while the remaining
    # longitudinal stem exists only close to the car.  Keep all line-proof
    # samples in that near field so the wide bar cannot erase the evidence for
    # the stem that leads into it.
    track_rows: tuple[float, ...] = (0.92, 0.86, 0.80)
    # Scan from the middle distance down to the terminal region independently
    # for a wide run.  The bar must be seen early enough for mark -> stem-loss
    # confirmation before the vehicle reaches it.
    # These rows are deliberately separate from ``track_rows``: the bar is
    # endpoint evidence, never a candidate left/right route.
    bar_rows: tuple[float, ...] = (0.38, 0.41, 0.44, 0.47, 0.50, 0.53, 0.56, 0.59, 0.62, 0.65, 0.68, 0.71, 0.74, 0.77, 0.80, 0.83)
    near_anchor_ratio: float = 0.90
    narrow_width_ratio: float = 0.18
    endpoint_width_ratio: float = 0.32
    endpoint_width_multiplier: float = 3.0
    endpoint_min_y_ratio: float = 0.35
    minimum_track_rows: int = 2
    maximum_center_spread_ratio: float = 0.10


@dataclass(frozen=True)
class ScanlineEvidence:
    confidence: float
    valid_line: bool
    line_lost: bool
    line_center_x: float | None
    line_centers: tuple[tuple[int, float, int], ...]
    endpoint_detected: bool
    endpoint_y: int | None
    endpoint_width: int | None
    normal_tape_width: float | None
    # ---- Skeleton-augmented fields (HybridScanlineAnalyzer) ----
    lookahead_x: float | None = None
    lookahead_y: int | None = None
    path_length_px: int = 0
    junction_detected: bool = False
    junction_y: int | None = None
    junction_arm_count: int = 0


@dataclass(frozen=True)
class ScanlineResult:
    evidence: ScanlineEvidence
    component_mask: np.ndarray


class TurnaroundState(str, Enum):
    FOLLOW_STRAIGHT = "FOLLOW_STRAIGHT"
    EARLY_BAR_PREDICTED = "EARLY_BAR_PREDICTED"
    BAR_MARKED = "BAR_MARKED"
    BRAKE_BEFORE_PIVOT = "BRAKE_BEFORE_PIVOT"
    PIVOT_180 = "PIVOT_180"
    STOP = "STOP"


@dataclass(frozen=True)
class TurnaroundConfig:
    endpoint_confirm_frames: int = 2
    line_lost_confirm_frames: int = 3
    reacquire_confirm_frames: int = 3
    minimum_confidence: float = 0.55
    bar_mark_timeout_seconds: float = 2.0
    brake_seconds: float = 0.15
    pivot_min_seconds: float = 2.5
    pivot_max_seconds: float = 5.0
    # ---- Hybrid early-prediction fields ----
    early_junction_min_y_ratio: float = 0.60
    junction_confirm_frames: int = 2


@dataclass(frozen=True)
class TurnaroundDecision:
    state: TurnaroundState
    reason: str
    endpoint_frames: int
    line_lost_frames: int
    reacquire_frames: int
    pivot_elapsed_seconds: float | None
    # ---- Hybrid ----
    junction_frames: int = 0


class IShapeScanlineAnalyzer:
    def __init__(self, config: ScanlineConfig = ScanlineConfig()) -> None:
        self.config = config

    @staticmethod
    def _row_run(mask: np.ndarray, y: int) -> tuple[float, int] | None:
        xs = np.flatnonzero(mask[y] > 0)
        if xs.size == 0:
            return None
        breaks = np.flatnonzero(np.diff(xs) > 1)
        starts = np.r_[0, breaks + 1]
        ends = np.r_[breaks, xs.size - 1]
        groups = [(int(xs[start]), int(xs[end])) for start, end in zip(starts, ends)]
        left, right = max(groups, key=lambda group: group[1] - group[0])
        return (left + right) / 2.0, right - left + 1

    def _near_component(self, binary: np.ndarray) -> np.ndarray:
        height, width = binary.shape
        count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        candidates: list[tuple[float, int]] = []
        anchor_y = int(round(height * self.config.near_anchor_ratio))
        for label in range(1, count):
            x, y, component_width, component_height, area = stats[label]
            if area < 20 or y + component_height < anchor_y:
                continue
            centre_distance = abs((x + component_width / 2.0) - width / 2.0) / max(1, width)
            candidates.append((centre_distance - min(0.20, area / max(1, width * height)), label))
        if not candidates:
            return np.zeros_like(binary)
        return np.where(labels == min(candidates)[1], 255, 0).astype(np.uint8)

    def analyze(self, frame: np.ndarray) -> ScanlineResult:
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _threshold, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        component = self._near_component(binary)
        rows: list[tuple[int, float, int]] = []
        for ratio in self.config.track_rows:
            y = min(height - 1, max(0, int(round(height * ratio))))
            run = self._row_run(component, y)
            if run is not None:
                rows.append((y, run[0], run[1]))
        narrow = [item for item in rows if item[2] <= width * self.config.narrow_width_ratio]
        normal_width = float(statistics.median(item[2] for item in narrow)) if narrow else None
        centers = tuple(narrow)
        if len(centers) >= self.config.minimum_track_rows:
            center_values = [item[1] for item in centers]
            spread = max(center_values) - min(center_values)
            valid_line = spread <= width * self.config.maximum_center_spread_ratio
            confidence = min(1.0, len(centers) / len(self.config.track_rows)) * (1.0 - min(1.0, spread / max(1.0, width * self.config.maximum_center_spread_ratio)))
            center_x = float(statistics.median(center_values))
        else:
            valid_line, confidence, center_x = False, 0.0, None
        endpoint_y: int | None = None
        endpoint_width: int | None = None
        required_width = max(width * self.config.endpoint_width_ratio, (normal_width or 1.0) * self.config.endpoint_width_multiplier)
        for ratio in self.config.bar_rows:
            y = min(height - 1, max(0, int(round(height * ratio))))
            run = self._row_run(component, y)
            if run is not None and y >= height * self.config.endpoint_min_y_ratio and run[1] >= required_width:
                endpoint_y, endpoint_width = y, run[1]
                break
        evidence = ScanlineEvidence(
            confidence=float(max(0.0, confidence)), valid_line=valid_line, line_lost=not valid_line,
            line_center_x=center_x, line_centers=centers, endpoint_detected=endpoint_y is not None,
            endpoint_y=endpoint_y, endpoint_width=endpoint_width, normal_tape_width=normal_width,
        )
        return ScanlineResult(evidence, component)


# ---------------------------------------------------------------------------
# Hybrid analyzer: skeleton + junction geometry from the old detector,
# combined with the new algorithm's bar/path separation architecture.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class HybridScanlineConfig(ScanlineConfig):
    """Extended config for the hybrid skeleton-augmented analyzer."""
    # --- Preprocessing ---
    blur_kernel: int = 5
    morphology_kernel: int = 5
    # --- Route component selection (from old detector) ---
    # Keep enough of a distant transverse bar for width evidence.  Near
    # anchoring still decides which connected component is the route.
    route_corridor_top_width_ratio: float = 0.80
    route_corridor_bottom_width_ratio: float = 0.95
    route_anchor_near_ratio: float = 0.32
    route_minimum_vertical_coverage: float = 0.20
    route_connection_kernel: int = 11
    # --- Marker detection (transverse bar) ---
    marker_minimum_arm_pixels: int = 18
    marker_scan_gap_pixels: int = 2
    marker_minimum_width_ratio: float = 2.5
    marker_max_local_turn_degrees: float = 30.0
    # Unlike the legacy fixed rows, search every row in this band.  A thin bar
    # must not disappear merely because it falls between two configured rows.
    bar_search_min_y_ratio: float = 0.20
    bar_search_max_y_ratio: float = 0.92
    bar_min_thickness_px: int = 4
    # --- Skeleton update rate ---
    route_path_update_frames: int = 2
    # --- Junction early-prediction: only junctions above this y ratio count ---
    early_junction_max_y_ratio: float = 0.88


class HybridScanlineAnalyzer:
    """Scanline + skeleton: bar detection uses junction topology and
    cross-track normals; path following uses near-anchored scanlines.

    The bar is endpoint evidence, NEVER a candidate left/right route.
    This is the same bar/path separation as the original scanline analyzer,
    but the bar detection is rotation-invariant (skeleton junction + normal-run)
    and can predict the bar from farther away (junction in the far skeleton).
    """

    def __init__(self, config: HybridScanlineConfig = HybridScanlineConfig()) -> None:
        self.config = config
        self._frame_index = 0
        self._cached_path: tuple[tuple[int, int], ...] = ()
        self._previous_route_path: tuple[tuple[int, int], ...] = ()

    @staticmethod
    def _row_run(mask: np.ndarray, y: int) -> tuple[float, int] | None:
        """Find the widest contiguous white segment on row y. (same as legacy)"""
        xs = np.flatnonzero(mask[y] > 0)
        if xs.size == 0:
            return None
        breaks = np.flatnonzero(np.diff(xs) > 1)
        starts = np.r_[0, breaks + 1]
        ends = np.r_[breaks, xs.size - 1]
        groups = [(int(xs[start]), int(xs[end])) for start, end in zip(starts, ends)]
        left, right = max(groups, key=lambda group: group[1] - group[0])
        return (left + right) / 2.0, right - left + 1

    # ------------------------------------------------------------------
    # Preprocessing (from old detector._make_mask)
    # ------------------------------------------------------------------

    @staticmethod
    def _make_mask(frame: np.ndarray, blur_kernel: int, morphology_kernel: int) -> np.ndarray:
        """Otsu threshold + morphological cleanup on the full frame ROI."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
        _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morphology_kernel, morphology_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # ------------------------------------------------------------------
    # Route corridor (from old detector._route_corridor_mask)
    # ------------------------------------------------------------------

    def _route_corridor_mask(self, shape: tuple[int, int]) -> np.ndarray:
        """Trapezoid mask: only consider pixels in the plausible driving corridor."""
        height, width = shape
        corridor = np.full((height, width), 255, dtype=np.uint8)
        centre = width // 2
        top_half = int(round(width * self.config.route_corridor_top_width_ratio / 2))
        bottom_half = int(round(width * self.config.route_corridor_bottom_width_ratio / 2))
        polygon = np.asarray(
            [
                (centre - top_half, 0),
                (centre + top_half, 0),
                (centre + bottom_half, height - 1),
                (centre - bottom_half, height - 1),
            ],
            dtype=np.int32,
        )
        corridor.fill(0)
        cv2.fillPoly(corridor, [polygon], 255)
        return corridor

    # ------------------------------------------------------------------
    # Component selection (from old detector._select_route_component)
    # ------------------------------------------------------------------

    def _select_route_component(self, mask: np.ndarray) -> np.ndarray:
        """Keep the continuous component anchored near the vehicle.

        Unlike the legacy _near_component, this adds corridor masking,
        gap closing, vertical coverage checks, and a weighted scoring
        function that prefers large centered components reaching the
        near floor area.
        """
        constrained = cv2.bitwise_and(mask, self._route_corridor_mask(mask.shape))
        # Close small gaps before component selection; near anchoring still
        # rejects separate objects like chair legs and wheels.
        connection_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.route_connection_kernel, self.config.route_connection_kernel),
        )
        constrained = cv2.morphologyEx(constrained, cv2.MORPH_CLOSE, connection_kernel)
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(constrained, connectivity=8)
        height, width = constrained.shape
        near_start = int(round(height * (1.0 - self.config.route_anchor_near_ratio)))
        near_labels = labels[near_start:, :]
        best_label = 0
        best_score = float("-inf")
        for label in range(1, count):
            x, y, component_width, component_height, area = stats[label]
            vertical_coverage = component_height / max(1, height)
            if vertical_coverage < self.config.route_minimum_vertical_coverage:
                continue
            near_count = int(np.count_nonzero(near_labels == label))
            if near_count == 0:
                continue
            centre_distance = abs((x + component_width / 2) - width / 2) / max(1, width)
            # Near anchoring dominates; large continuous tape then wins over a
            # small object at a similar position.
            score = near_count * 4.0 + area * 0.25 - centre_distance * area * 0.10
            if score > best_score:
                best_label, best_score = label, score
        if best_label == 0:
            return np.zeros_like(constrained)
        return np.where(labels == best_label, 255, 0).astype(np.uint8)

    # ------------------------------------------------------------------
    # Skeletonization (from old detector._skeletonize)
    # ------------------------------------------------------------------

    @staticmethod
    def _skeletonize(mask: np.ndarray) -> np.ndarray:
        """Return a one-pixel route skeleton without opencv-contrib."""
        if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
            return cv2.ximgproc.thinning(mask)
        working = mask.copy()
        skeleton = np.zeros_like(mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        while cv2.countNonZero(working):
            eroded = cv2.erode(working, kernel)
            edge = cv2.subtract(working, cv2.dilate(eroded, kernel))
            skeleton = cv2.bitwise_or(skeleton, edge)
            working = eroded
        return skeleton

    # ------------------------------------------------------------------
    # Skeleton BFS: y-min endpoint selection (instead of longest-distance)
    # ------------------------------------------------------------------

    def _trace_skeleton(
        self,
        skeleton: np.ndarray,
    ) -> tuple[tuple[tuple[int, int], ...], tuple[int, int] | None, int]:
        """BFS from the near-centre skeleton point.  Return the path to the
        endpoint with the SMALLEST y (farthest up in the image), its lookahead
        point, and the total distance.

        At a T-junction the longitudinal stem endpoint is always higher (smaller
        y) than the transverse bar endpoints, so ``min(endpoints, key=y)``
        naturally selects the stem — the bar is never followed.
        """
        coordinates = np.argwhere(skeleton > 0)  # (y, x)
        if coordinates.size == 0:
            return (), None, 0

        height, width = skeleton.shape
        lowest_y = int(coordinates[:, 0].max())
        start_options = coordinates[coordinates[:, 0] >= lowest_y - 2]
        start_row = min(start_options, key=lambda item: abs(int(item[1]) - width // 2))
        start = (int(start_row[0]), int(start_row[1]))
        nodes = [(int(y), int(x)) for y, x in coordinates]
        node_index = {node: index for index, node in enumerate(nodes)}
        start_index = node_index[start]

        neighbours: list[list[int]] = [[] for _ in nodes]
        for index, (y, x) in enumerate(nodes):
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    other = node_index.get((y + dy, x + dx))
                    if other is not None:
                        neighbours[index].append(other)

        distances = [-1] * len(nodes)
        parents = [-1] * len(nodes)
        distances[start_index] = 0
        queue: deque[int] = deque([start_index])
        while queue:
            current = queue.popleft()
            for other in neighbours[current]:
                if distances[other] >= 0:
                    continue
                distances[other] = distances[current] + 1
                parents[other] = current
                queue.append(other)

        # All degree <= 2 (non-junction) reachable skeleton pixels.
        endpoints = [
            index
            for index, linked in enumerate(neighbours)
            if index != start_index and len(linked) <= 2 and distances[index] >= 0
        ]
        if not endpoints:
            # Fallback: use any reachable node.
            reachable = [i for i, d in enumerate(distances) if d >= 0]
            if not reachable:
                return (), None, 0
            endpoints = reachable

        # ---- KEY DIFFERENCE from old algorithm ----
        # Select the endpoint with the SMALLEST y (farthest up / farthest
        # ahead).  At a T-junction the stem tip is higher than the bar tips,
        # so the bar is never selected as the path.
        end_index = min(endpoints, key=lambda i: nodes[i][0])

        def reconstruct(end_idx: int) -> tuple[tuple[int, int], ...]:
            ordered: list[tuple[int, int]] = []
            while end_idx >= 0:
                y, x = nodes[end_idx]
                ordered.append((x, y))
                end_idx = parents[end_idx]
            ordered.reverse()
            return tuple(ordered)

        path = reconstruct(end_index)
        path_length = distances[end_index]

        # Lookahead: point ~60% along the path, as in the old detector.
        if len(path) >= 2:
            lookahead_index = min(
                len(path) - 1,
                max(1, int(round((len(path) - 1) * 0.60))),
            )
            lookahead = path[lookahead_index]
        else:
            lookahead = path[0] if path else None

        return path, lookahead, path_length

    # ------------------------------------------------------------------
    # Junction detection (from old detector._marker_evidence topology layer)
    # ------------------------------------------------------------------

    @staticmethod
    def _detect_junction(skeleton: np.ndarray) -> tuple[bool, int | None, int]:
        """Find the highest (smallest-y) skeleton junction with >= 3 neighbours.

        Returns (detected, junction_y, arm_count).
        A T-junction has arm_count == 3; an X-cross has arm_count >= 4.
        """
        binary = (skeleton > 0).astype(np.uint8)
        if not binary.any():
            return False, None, 0
        neighbour_count = cv2.filter2D(binary, cv2.CV_16S, np.ones((3, 3), dtype=np.int16)) - binary
        junctions = np.where((binary > 0) & (neighbour_count >= 3), 255, 0).astype(np.uint8)
        count, _labels, stats, centroids = cv2.connectedComponentsWithStats(junctions, connectivity=8)
        best: tuple[int, int, int] | None = None  # (y, arm_count, -area)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 2:
                continue
            cx, cy = centroids[label]
            # Count arms after removing the junction patch.
            removed = binary.copy()
            cv2.circle(removed, (int(round(cx)), int(round(cy))), 5, 0, -1)
            arm_count_labels, _arm_labels, arm_stats, _arm_centroids = (
                cv2.connectedComponentsWithStats(removed, connectivity=8)
            )
            arms = sum(
                1
                for la in range(1, arm_count_labels)
                if arm_stats[la, cv2.CC_STAT_AREA] >= 4
            )
            if arms < 3:
                continue
            if best is None or int(cy) < best[0]:
                best = (int(cy), arms, -area)
        if best is None:
            return False, None, 0
        return True, best[0], best[1]

    # ------------------------------------------------------------------
    # Normal run-length (from old detector._normal_run_length)
    # ------------------------------------------------------------------

    @staticmethod
    def _normal_run_length(
        binary: np.ndarray,
        x: float,
        y: float,
        dx: float,
        dy: float,
        *,
        gap_limit: int,
        maximum_length: int = 140,
    ) -> int:
        """Measure how far white tape continues from (x, y) along (dx, dy)."""
        height, width = binary.shape
        last_white, gaps = 0, 0
        for distance in range(1, maximum_length + 1):
            sample_x = int(round(x + dx * distance))
            sample_y = int(round(y + dy * distance))
            if not (0 <= sample_x < width and 0 <= sample_y < height):
                break
            if binary[sample_y, sample_x]:
                last_white, gaps = distance, 0
            else:
                gaps += 1
                if gaps > gap_limit:
                    break
        return last_white

    # ------------------------------------------------------------------
    # Transverse marker via cross-track normals (from old detector)
    # ------------------------------------------------------------------

    def _transverse_marker_evidence(
        self,
        mask: np.ndarray,
        path: tuple[tuple[int, int], ...],
    ) -> tuple[bool, tuple[int, int] | None, int]:
        """Detect a transverse bar by measuring cross-track white runs along
        the path normal.  Rotation-invariant — works regardless of bar angle.
        """
        if len(path) < 9:
            return False, None, 0
        binary = (mask > 0).astype(np.uint8)
        # Estimate normal tape width from distance transform medians.
        distance = cv2.distanceTransform(binary, cv2.DIST_L2, 3)
        route_half_widths = [
            float(distance[y, x])
            for x, y in path
            if 0 <= y < distance.shape[0]
            and 0 <= x < distance.shape[1]
            and distance[y, x] > 0
        ]
        normal_half_width = float(np.median(route_half_widths)) if route_half_widths else 1.0
        required_arm = max(
            self.config.marker_minimum_arm_pixels,
            int(np.ceil(normal_half_width * self.config.marker_minimum_width_ratio)),
        )
        best_score = 0
        best_point: tuple[int, int] | None = None
        for index in range(4, len(path) - 4, 3):
            x, y = path[index]
            before = np.asarray(path[index - 4], dtype=np.float32)
            current = np.asarray(path[index], dtype=np.float32)
            after = np.asarray(path[index + 4], dtype=np.float32)
            tangent = after - before
            length = float(np.linalg.norm(tangent))
            if length < 8:
                continue
            incoming = current - before
            outgoing = after - current
            incoming_len = float(np.linalg.norm(incoming))
            outgoing_len = float(np.linalg.norm(outgoing))
            if incoming_len < 3 or outgoing_len < 3:
                continue
            cosine = float(np.clip(
                np.dot(incoming, outgoing) / (incoming_len * outgoing_len), -1.0, 1.0
            ))
            local_turn = float(np.degrees(np.arccos(cosine)))
            if local_turn > self.config.marker_max_local_turn_degrees:
                continue
            normal_x, normal_y = -float(tangent[1]) / length, float(tangent[0]) / length
            left = self._normal_run_length(
                binary, x, y, normal_x, normal_y,
                gap_limit=self.config.marker_scan_gap_pixels,
            )
            right = self._normal_run_length(
                binary, x, y, -normal_x, -normal_y,
                gap_limit=self.config.marker_scan_gap_pixels,
            )
            if min(left, right) < required_arm:
                continue
            score = left + right
            if score > best_score:
                best_score = score
                best_point = (int(x), int(y))
        return best_point is not None, best_point, best_score

    def _find_wide_bar(
        self,
        component: np.ndarray,
        normal_width: float | None,
    ) -> tuple[int | None, int | None]:
        """Find the first real transverse run without relying on fixed rows.

        Adjacent wide rows are grouped so a single noisy floor row cannot be
        a bar.  The topmost group is chosen because it is the earliest visible
        incoming transverse feature.
        """
        height, width = component.shape
        required_width = max(
            width * self.config.endpoint_width_ratio,
            (normal_width or 1.0) * self.config.endpoint_width_multiplier,
        )
        start = max(0, int(round(height * self.config.bar_search_min_y_ratio)))
        end = min(height - 1, int(round(height * self.config.bar_search_max_y_ratio)))
        groups: list[list[tuple[int, int]]] = []
        current: list[tuple[int, int]] = []
        for y in range(start, end + 1):
            run = self._row_run(component, y)
            if run is not None and run[1] >= required_width:
                current.append((y, run[1]))
            elif current:
                groups.append(current)
                current = []
        if current:
            groups.append(current)
        groups = [group for group in groups if len(group) >= self.config.bar_min_thickness_px]
        if not groups:
            return None, None
        first = groups[0]
        best_y, best_width = max(first, key=lambda item: item[1])
        return best_y, best_width

    # ------------------------------------------------------------------
    # Hybrid analyze(): combine scanline path + skeleton prediction
    # ------------------------------------------------------------------

    def analyze(self, frame: np.ndarray) -> ScanlineResult:
        height, width = frame.shape[:2]
        self._frame_index += 1

        # --- Step 1: Mask + component selection (old detector pipeline) ---
        mask = self._make_mask(frame, self.config.blur_kernel, self.config.morphology_kernel)
        component = self._select_route_component(mask)

        # --- Step 2: Scanline path following (same as legacy) ---
        rows: list[tuple[int, float, int]] = []
        for ratio in self.config.track_rows:
            y = min(height - 1, max(0, int(round(height * ratio))))
            run = self._row_run(component, y)
            if run is not None:
                rows.append((y, run[0], run[1]))
        narrow = [item for item in rows if item[2] <= width * self.config.narrow_width_ratio]
        normal_width = float(statistics.median(item[2] for item in narrow)) if narrow else None
        centers = tuple(narrow)
        if len(centers) >= self.config.minimum_track_rows:
            center_values = [item[1] for item in centers]
            spread = max(center_values) - min(center_values)
            valid_line = spread <= width * self.config.maximum_center_spread_ratio
            confidence = min(1.0, len(centers) / len(self.config.track_rows)) * (
                1.0 - min(1.0, spread / max(1.0, width * self.config.maximum_center_spread_ratio))
            )
            center_x = float(statistics.median(center_values))
        else:
            valid_line, confidence, center_x = False, 0.0, None

        # --- Step 3: Dense width profile, replacing fixed bar_rows ---
        # Keep a far bar as prediction only; it becomes an endpoint only once
        # it reaches the near/middle safety band.
        wide_bar_y, wide_bar_width = self._find_wide_bar(component, normal_width)
        endpoint_y = wide_bar_y if wide_bar_y is not None and wide_bar_y >= height * self.config.endpoint_min_y_ratio else None
        endpoint_width = wide_bar_width if endpoint_y is not None else None

        # --- Step 4: Skeleton analysis (every route_path_update_frames) ---
        lookahead_x: float | None = None
        lookahead_y: int | None = None
        path_length_px = 0
        junction_detected = False
        junction_y: int | None = None
        junction_arm_count = 0

        should_update_path = (
            not self._cached_path
            or self._frame_index % self.config.route_path_update_frames == 0
        )
        if should_update_path and component.any():
            skeleton = self._skeletonize(component)
            path, lookahead, path_len = self._trace_skeleton(skeleton)
            self._cached_path = path
            self._previous_route_path = path
            path_length_px = path_len

            if lookahead is not None:
                lookahead_x = float(lookahead[0])
                lookahead_y = int(lookahead[1])

            # Junction: detects T-junctions (3 arms) and X-crosses (4+ arms).
            j_detected, j_y, j_arms = self._detect_junction(skeleton)
            if j_detected and j_y is not None:
                # Only accept junctions in the upper portion of the image
                # (far field).  Near-field junctions are likely noise, not
                # a real approaching bar.
                if j_y < height * self.config.early_junction_max_y_ratio:
                    junction_detected = True
                    junction_y = j_y
                    junction_arm_count = j_arms

            # A wide transverse run in the far field is an early prediction
            # even if skeleton thinning did not leave a clean 3-arm pixel.
            if wide_bar_y is not None and wide_bar_y < height * self.config.endpoint_min_y_ratio:
                junction_detected = True
                junction_y = wide_bar_y
                junction_arm_count = max(junction_arm_count, 0)

            # --- Step 4b: Transverse marker via cross-track normals ---
            # If skeleton junction was found, confirm with normal-run evidence.
            # If junction was NOT found but the path has enough pixels, still
            # run the normal-run check as a direct bar detector (catches cases
            # where the junction is smoothed away by morphology).
            if path and len(path) >= 9:
                marker_detected, marker_point, marker_width = self._transverse_marker_evidence(component, path)
                if marker_detected and marker_point is not None:
                    marker_y = marker_point[1]
                    if marker_y >= height * self.config.endpoint_min_y_ratio:
                        # Rotation-invariant near/middle confirmation.
                        endpoint_y = marker_y
                        endpoint_width = max(endpoint_width or 0, marker_width)
                    elif marker_y < height * self.config.early_junction_max_y_ratio:
                        # Rotation-invariant far prediction; keep driving.
                        junction_detected = True
                        junction_y = marker_y
        elif self._cached_path:
            path_length_px = len(self._cached_path)

        # --- Step 5: Assemble evidence ---
        evidence = ScanlineEvidence(
            confidence=float(max(0.0, confidence)),
            valid_line=valid_line,
            line_lost=not valid_line,
            line_center_x=center_x,
            line_centers=centers,
            endpoint_detected=endpoint_y is not None,
            endpoint_y=endpoint_y,
            endpoint_width=endpoint_width,
            normal_tape_width=normal_width,
            # Skeleton-augmented fields:
            lookahead_x=lookahead_x,
            lookahead_y=lookahead_y,
            path_length_px=path_length_px,
            junction_detected=junction_detected,
            junction_y=junction_y,
            junction_arm_count=junction_arm_count,
        )
        return ScanlineResult(evidence, component)


# ---------------------------------------------------------------------------
# Extended planner: supports EARLY_BAR_PREDICTED from hybrid junction evidence.
# ---------------------------------------------------------------------------

class IShapeTurnaroundPlanner:
    def __init__(self, config: TurnaroundConfig = TurnaroundConfig()) -> None:
        self.config = config
        self.state = TurnaroundState.FOLLOW_STRAIGHT
        self._endpoint_frames = 0
        self._line_lost_frames = 0
        self._reacquire_frames = 0
        self._bar_marked_at: float | None = None
        self._brake_started_at: float | None = None
        self._pivot_started_at: float | None = None
        # ---- Hybrid ----
        self._junction_frames = 0

    def step(self, evidence: ScanlineEvidence, now: float) -> TurnaroundDecision:
        usable = evidence.valid_line and evidence.confidence >= self.config.minimum_confidence

        # ================================================================
        # FOLLOW_STRAIGHT
        # ================================================================
        if self.state is TurnaroundState.FOLLOW_STRAIGHT:
            # ---- Junction early prediction (hybrid) ----
            self._junction_frames = (
                self._junction_frames + 1
                if usable and evidence.junction_detected
                else 0
            )
            if self._junction_frames >= self.config.junction_confirm_frames:
                self.state = TurnaroundState.EARLY_BAR_PREDICTED
                self._endpoint_frames = 0
                self._line_lost_frames = 0
                return TurnaroundDecision(
                    self.state,
                    f"junction_at_y={evidence.junction_y}_early_bar_predicted",
                    self._endpoint_frames, 0, 0, None,
                    junction_frames=self._junction_frames,
                )

            # ---- Standard endpoint detection (legacy + hybrid fallback) ----
            self._endpoint_frames = (
                self._endpoint_frames + 1
                if usable and evidence.endpoint_detected
                else 0
            )
            if self._endpoint_frames >= self.config.endpoint_confirm_frames:
                self.state = TurnaroundState.BAR_MARKED
                self._bar_marked_at = now
                self._line_lost_frames = 0
                return TurnaroundDecision(
                    self.state,
                    "lower_transverse_bar_marked_follow_until_stem_lost",
                    self._endpoint_frames, 0, 0, None,
                )
            return TurnaroundDecision(
                self.state,
                "following_near_anchored_longitudinal_line",
                self._endpoint_frames, 0, 0, None,
            )

        # ================================================================
        # EARLY_BAR_PREDICTED (hybrid): junction seen in far field.
        # Keep driving straight; transition to BAR_MARKED when either
        # endpoint_detected fires or line is lost (stem disappeared).
        # False alarm recovery if junction disappears.
        # ================================================================
        if self.state is TurnaroundState.EARLY_BAR_PREDICTED:
            # Confirm: if the bar arrives at bar_rows while we're waiting.
            self._endpoint_frames = (
                self._endpoint_frames + 1
                if usable and evidence.endpoint_detected
                else 0
            )
            if self._endpoint_frames >= self.config.endpoint_confirm_frames:
                self.state = TurnaroundState.BAR_MARKED
                self._bar_marked_at = now
                self._line_lost_frames = 0
                return TurnaroundDecision(
                    self.state,
                    "early_prediction_confirmed_by_bar_rows_endpoint",
                    self._endpoint_frames, 0, 0, None,
                )

            # Confirm: stem lost (vehicle crossed the bar before bar_rows
            # detected it — can happen with fast approach).
            self._line_lost_frames = (
                self._line_lost_frames + 1 if evidence.line_lost else 0
            )
            if self._line_lost_frames >= self.config.line_lost_confirm_frames:
                self.state = TurnaroundState.BRAKE_BEFORE_PIVOT
                self._brake_started_at = now
                return TurnaroundDecision(
                    self.state,
                    "stem_lost_from_early_prediction_braking",
                    self._endpoint_frames, self._line_lost_frames, 0, 0.0,
                )

            # False alarm check: junction disappeared and no bar confirmed.
            if not evidence.junction_detected and not evidence.endpoint_detected:
                self._junction_frames = max(0, self._junction_frames - 1)
                if self._junction_frames == 0:
                    self.state = TurnaroundState.FOLLOW_STRAIGHT
                    self._endpoint_frames = 0
                    self._line_lost_frames = 0
                    return TurnaroundDecision(
                        self.state,
                        "early_prediction_false_alarm_junction_lost",
                        0, 0, 0, None,
                    )

            return TurnaroundDecision(
                self.state,
                f"early_bar_predicted_waiting_for_stem_loss_or_endpoint_confirm_junction_y={evidence.junction_y}",
                self._endpoint_frames, self._line_lost_frames, 0, None,
            )

        # ================================================================
        # BAR_MARKED
        # ================================================================
        if self.state is TurnaroundState.BAR_MARKED:
            marked_at = self._bar_marked_at if self._bar_marked_at is not None else now
            if now - marked_at >= self.config.bar_mark_timeout_seconds:
                self.state = TurnaroundState.FOLLOW_STRAIGHT
                self._endpoint_frames = 0
                self._line_lost_frames = 0
                self._junction_frames = 0
                return TurnaroundDecision(
                    self.state, "bar_mark_timeout_returning_to_follow", 0, 0, 0, None,
                )
            # A missing near longitudinal stem means the car has passed the
            # bar.  Do not require the bar itself to remain visible.
            self._line_lost_frames = (
                self._line_lost_frames + 1 if evidence.line_lost else 0
            )
            if self._line_lost_frames >= self.config.line_lost_confirm_frames:
                self.state = TurnaroundState.BRAKE_BEFORE_PIVOT
                self._brake_started_at = now
                return TurnaroundDecision(
                    self.state,
                    "longitudinal_stem_lost_after_bar_braking",
                    self._endpoint_frames, self._line_lost_frames, 0, 0.0,
                )
            return TurnaroundDecision(
                self.state,
                "bar_marked_following_until_stem_lost",
                self._endpoint_frames, self._line_lost_frames, 0, None,
            )

        # ================================================================
        # BRAKE_BEFORE_PIVOT
        # ================================================================
        if self.state is TurnaroundState.BRAKE_BEFORE_PIVOT:
            started = self._brake_started_at if self._brake_started_at is not None else now
            if now - started < self.config.brake_seconds:
                return TurnaroundDecision(
                    self.state,
                    "braking_before_right_pivot",
                    self._endpoint_frames, self._line_lost_frames, 0, None,
                )
            self.state = TurnaroundState.PIVOT_180
            self._pivot_started_at = now
            self._reacquire_frames = 0
            return TurnaroundDecision(
                self.state,
                "brake_complete_starting_right_pivot",
                self._endpoint_frames, self._line_lost_frames, 0, 0.0,
            )

        # ================================================================
        # PIVOT_180
        # ================================================================
        if self.state is TurnaroundState.PIVOT_180:
            started = self._pivot_started_at if self._pivot_started_at is not None else now
            elapsed = max(0.0, now - started)
            if elapsed >= self.config.pivot_max_seconds:
                self.state = TurnaroundState.STOP
                return TurnaroundDecision(
                    self.state,
                    "pivot_timeout_without_longitudinal_reacquire",
                    self._endpoint_frames, self._line_lost_frames,
                    self._reacquire_frames, elapsed,
                )
            if elapsed < self.config.pivot_min_seconds:
                return TurnaroundDecision(
                    self.state,
                    "pivoting_minimum_time",
                    self._endpoint_frames, self._line_lost_frames, 0, elapsed,
                )
            self._reacquire_frames = (
                self._reacquire_frames + 1
                if usable and not evidence.endpoint_detected
                else 0
            )
            if self._reacquire_frames >= self.config.reacquire_confirm_frames:
                self.state = TurnaroundState.FOLLOW_STRAIGHT
                self._endpoint_frames = 0
                self._junction_frames = 0
                return TurnaroundDecision(
                    self.state,
                    "longitudinal_line_reacquired",
                    0, self._line_lost_frames, self._reacquire_frames, elapsed,
                )
            return TurnaroundDecision(
                self.state,
                "pivoting_until_longitudinal_reacquire",
                self._endpoint_frames, self._line_lost_frames,
                self._reacquire_frames, elapsed,
            )

        # ================================================================
        # STOP
        # ================================================================
        return TurnaroundDecision(
            self.state,
            "stopped_after_pivot_timeout",
            self._endpoint_frames, self._line_lost_frames,
            self._reacquire_frames, None,
        )
