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
    white_saturation_max: int = 82
    white_value_min: int = 168
    green_hue_min: int = 32
    green_hue_max: int = 96
    # Measured from the live CSI stream: carpet S median ~=157, wall ~=53,
    # pale floor ~=73.  Course membership is hue + saturation, not brightness.
    green_saturation_min: int = 95
    green_value_min: int = 0
    minimum_green_roi_ratio: float = 0.18
    roi_top_ratio: float = 0.38
    green_neighbour_kernel: int = 31
    # The usable course is the large green connected region near the vehicle,
    # not a fixed centre trapezoid.  This keeps side-entering turn tape while
    # excluding the room floor outside the mat.
    green_field_close_kernel: int = 25
    green_field_min_area_ratio: float = 0.025
    green_field_near_anchor_ratio: float = 0.72
    green_field_white_margin_pixels: int = 36
    # Green course selection is always anchored in the lower camera field.
    # Only that near field may be bridged across tape/red-marker holes; doing
    # it in the far/upper image can join the carpet to green-tinted walls.
    green_bridge_min_y_ratio: float = 0.62
    # The red outline is the saturation-derived green-field boundary.  White
    # candidates may not cross above that boundary into the wall/floor.
    # Keep this at zero: tape inside the mat is recovered by the field itself,
    # not by expanding the red boundary upward.
    tape_above_course_boundary_tolerance_pixels: int = 0
    # Do not make the HSV mask stricter.  Instead, after its permissive
    # candidate stage, require a route backbone to have green floor on both
    # sides of most of its locally measured widths.
    green_backbone_min_length: int = 100
    green_backbone_samples: int = 17
    green_backbone_min_supported_ratio: float = 0.58
    green_backbone_min_supported_samples: int = 6
    green_backbone_side_margin_pixels: int = 7
    green_backbone_green_probe_radius: int = 2
    # Measured on the 640x480 fisheye preview: the near tape's half-width is
    # below this for almost all of its backbone.  A broad pale floor patch
    # produces a much thicker distance-transform core.
    green_backbone_max_tape_half_width: float = 40.0
    green_backbone_max_wide_ratio: float = 0.30
    # The raw-green proof above is deliberately conservative.  At a close
    # junction the tape itself, a red marker, or a fish-eye stretched edge can
    # temporarily erase one of those raw-green probes even though the white
    # candidate is completely enclosed by the recovered green course field.
    # This second proof is only a fallback; it still rejects broad floor
    # patches by width and requires support on both sides of the skeleton.
    course_backbone_min_supported_ratio: float = 0.42
    course_backbone_min_supported_samples: int = 4
    course_backbone_max_tape_half_width: float = 58.0
    course_backbone_max_wide_ratio: float = 0.45
    red_excess_min: int = 44
    red_channel_min: int = 120
    red_hue_low_max: int = 12
    red_hue_high_min: int = 165
    red_saturation_min: int = 85
    red_value_min: int = 70
    red_min_component_area_ratio: float = 0.00015
    red_min_span_ratio: float = 0.18
    # A single physical red layer can be skewed by the 160-degree fisheye:
    # its left/right fragments need not have equal image Y.  The two physical
    # layers are roughly 10 cm apart and remain visibly farther apart than
    # this tolerance in the intended camera view.
    red_group_y_tolerance_ratio: float = 0.10
    red_centre_tolerance_ratio: float = 0.20


@dataclass(frozen=True)
class RedBandLayer:
    """One physical red-band layer reconstructed from one or more blobs."""
    y: int
    bottom_y: int
    span: int
    fragment_count: int
    y_spread: int


class GreenWhiteHybridScanlineAnalyzer(HybridScanlineAnalyzer):
    """Reuse all I-turn geometry; replace only black-tape Otsu segmentation."""
    def __init__(self, config: GreenWhiteScanlineConfig = GreenWhiteScanlineConfig()) -> None:
        super().__init__(config)
        self.config = config
        self._latest_green_mask: np.ndarray | None = None
        self._latest_green_field: np.ndarray | None = None
        self._latest_course_allowed: np.ndarray | None = None
        self._latest_tape_candidate_mask: np.ndarray | None = None
        self._latest_red_marker_mask: np.ndarray | None = None
        self._latest_tape_fit_line: tuple[tuple[int, int], tuple[int, int]] | None = None
        self._latest_red_band_layers: tuple[RedBandLayer, ...] = ()

    @staticmethod
    def _odd(value: int) -> int:
        return value if value % 2 else value + 1

    def _course_field_mask(self, green: np.ndarray) -> np.ndarray:
        """Return the green mat component anchored in the near camera field.

        White tape cuts small holes through the green fabric, so close those
        first.  Selecting one large lower connected component lets a turning
        line enter from either side of the image; unlike a fixed ROI, no
        centre position is assumed.
        """
        config = self.config
        close_size = self._odd(max(3, config.green_field_close_kernel))
        closed = cv2.morphologyEx(
            green,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size)),
        )
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(closed, connectivity=8)
        height, width = green.shape
        near_start = int(round(height * config.green_field_near_anchor_ratio))
        near_labels = labels[near_start:, :]
        minimum_area = int(round(width * height * config.green_field_min_area_ratio))
        best_label, best_score = 0, float("-inf")
        for label in range(1, count):
            _x, _y, _component_width, _component_height, area = map(int, stats[label])
            near_area = int(np.count_nonzero(near_labels == label))
            if area < minimum_area or near_area == 0:
                continue
            score = near_area * 4.0 + area
            if score > best_score:
                best_label, best_score = label, score
        if best_label == 0:
            return np.zeros_like(green)
        return np.where(labels == best_label, 255, 0).astype(np.uint8)

    @property
    def course_field_mask(self) -> np.ndarray | None:
        """Current-frame green course area, shared with the preview overlay."""
        return self._latest_green_field

    @property
    def red_marker_mask(self) -> np.ndarray | None:
        return self._latest_red_marker_mask

    @property
    def tape_candidate_mask(self) -> np.ndarray | None:
        """All white tape evidence inside the recovered course (display-only)."""
        return self._latest_tape_candidate_mask

    @property
    def tape_fit_line(self) -> tuple[tuple[int, int], tuple[int, int]] | None:
        """Diagnostic-only best straight segment through permissive tape pixels."""
        return self._latest_tape_fit_line

    @property
    def red_band_layers(self) -> tuple[RedBandLayer, ...]:
        """Visible red layers, ordered from far (small Y) to near (large Y)."""
        return self._latest_red_band_layers

    @staticmethod
    def _fit_permissive_tape(mask: np.ndarray) -> tuple[tuple[int, int], tuple[int, int]] | None:
        """Fit a display-only line before strict component validation.

        This is deliberately not fed into the motor controller.  It lets the
        preview show where permissive white evidence suggests tape may be when
        the stricter route component is correctly withheld.
        """
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180.0, 32, minLineLength=60, maxLineGap=32)
        if lines is None:
            return None
        x1, y1, x2, y2 = max(
            (tuple(map(int, line[0])) for line in lines),
            key=lambda line: float((line[2] - line[0]) ** 2 + (line[3] - line[1]) ** 2),
        )
        return (x1, y1), (x2, y2)

    @staticmethod
    def _below_course_top_boundary(field: np.ndarray, tolerance: int) -> np.ndarray:
        """Keep only pixels on/below each column's green-course top edge.

        This is intentionally asymmetric.  The problematic false positives
        are above the mat (wall/floor); the true tape lies on the mat below
        that edge.  It does not alter the green contour shown in red.
        """
        height, width = field.shape
        gate = np.zeros_like(field)
        for x in range(width):
            ys = np.flatnonzero(field[:, x] > 0)
            if ys.size:
                gate[max(0, int(ys[0]) - tolerance):, x] = 255
        return gate

    def _make_mask(self, frame: np.ndarray, blur_kernel: int, morphology_kernel: int) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        config = self.config
        # Do not leave a previous frame's diagnostic overlay visible if the
        # current frame has no valid green course field.
        self._latest_course_allowed = None
        self._latest_tape_candidate_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        self._latest_red_marker_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        self._latest_tape_fit_line = None
        self._latest_red_band_layers = ()
        white = cv2.inRange(
            hsv,
            np.array((0, 0, config.white_value_min), dtype=np.uint8),
            np.array((180, config.white_saturation_max, 255), dtype=np.uint8),
        )
        green_hsv = cv2.inRange(
            hsv,
            np.array((config.green_hue_min, config.green_saturation_min, config.green_value_min), dtype=np.uint8),
            np.array((config.green_hue_max, 255, 255), dtype=np.uint8),
        )
        # The carpet boundary is determined by green hue and saturation only.
        # Do not use V/brightness or RGB channel magnitude: exposure changes
        # those strongly between the mat, wall, and fish-eye edge.
        green = green_hsv
        # `_select_route_component` runs immediately after `_make_mask` in
        # the inherited analyzer.  Retain raw green for side probes, and its
        # connected course field for the spatial candidate gate.
        self._latest_green_mask = green
        # White tape and red markers physically cover strips of the carpet.
        # Treat those colour-confirmed strips as *bridges* only while
        # recovering the connected course field: otherwise a wide near tape
        # can split the lower carpet from the anchored upper carpet and make
        # its own white pixels ineligible on the next line.  We do not bridge
        # arbitrary dark/bright floor pixels.
        blue, green_channel, red_channel = cv2.split(frame)
        red_excess = red_channel.astype(np.int16) - np.maximum(green_channel, blue).astype(np.int16)
        red_bridge = np.where(
            (red_channel >= config.red_channel_min) & (red_excess >= config.red_excess_min),
            255,
            0,
        ).astype(np.uint8)
        # A bridge must begin next to real green carpet.  This prevents a
        # red object or bright room floor from manufacturing an entire course
        # when no green field exists at all, while the generous radius still
        # spans the widest close tape in the fisheye view.
        bridge_radius = self._odd(max(3, config.green_field_white_margin_pixels * 4 + 1))
        green_neighbourhood = cv2.dilate(
            green,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bridge_radius, bridge_radius)),
        )
        bridge = cv2.bitwise_and(cv2.bitwise_or(white, red_bridge), green_neighbourhood)
        bridge[:int(round(frame.shape[0] * config.green_bridge_min_y_ratio)), :] = 0
        course_seed = cv2.bitwise_or(green, bridge)
        green_field = self._course_field_mask(course_seed)
        self._latest_green_field = green_field
        if not np.any(green_field):
            return np.zeros_like(white)
        # Tape replaces the green beneath it.  Permit it inside the recovered
        # mat field or within a tape-width margin of it, regardless of whether
        # it appears left, right, or centre during a turn.
        margin = self._odd(max(3, config.green_field_white_margin_pixels * 2 + 1))
        course_allowed = cv2.dilate(
            green_field,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (margin, margin)),
        )
        self._latest_course_allowed = course_allowed
        mask = cv2.bitwise_and(white, course_allowed)
        cleanup_size = self._odd(max(3, morphology_kernel))
        cleanup = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cleanup_size, cleanup_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cleanup)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cleanup)
        # The dilated course allowance retains tape holes, but it also reaches
        # a little above the physical carpet boundary.  Remove only those
        # above-boundary pixels: yellow overlay and all downstream route/bar
        # logic now agree that wall pixels are out of scope.
        top_gate = self._below_course_top_boundary(
            green_field,
            config.tape_above_course_boundary_tolerance_pixels,
        )
        mask = cv2.bitwise_and(mask, top_gate)
        self._latest_tape_candidate_mask = mask
        self._latest_tape_fit_line = self._fit_permissive_tape(mask)
        return mask

    @staticmethod
    def _green_at(green: np.ndarray, x: float, y: float, radius: int) -> bool:
        height, width = green.shape
        centre_x, centre_y = int(round(x)), int(round(y))
        left, right = max(0, centre_x - radius), min(width, centre_x + radius + 1)
        top, bottom = max(0, centre_y - radius), min(height, centre_y + radius + 1)
        return left < right and top < bottom and bool(np.any(green[top:bottom, left:right]))

    def _backbone_supported_by(
        self,
        component: np.ndarray,
        support_mask: np.ndarray | None,
        *,
        minimum_ratio: float,
        minimum_samples: int,
        maximum_half_width: float,
        maximum_wide_ratio: float,
    ) -> tuple[bool, float]:
        """Check the selected component's centreline, not every white pixel.

        A real tape has a narrow skeleton with green floor on both sides of
        that skeleton.  A bright floor patch can touch the mat along one edge,
        but its long skeleton path has no such two-sided support.  The probe
        distance is derived from the local component width, so fish-eye
        widening and oblique tape remain valid.
        """
        if support_mask is None or support_mask.shape != component.shape:
            return False, 0.0
        skeleton = self._skeletonize(component)
        path, _lookahead, path_length = self._trace_skeleton(skeleton)
        if path_length < self.config.green_backbone_min_length or len(path) < 3:
            return False, 0.0
        distance = cv2.distanceTransform((component > 0).astype(np.uint8), cv2.DIST_L2, 3)
        sample_count = min(self.config.green_backbone_samples, max(3, len(path) - 2))
        indices = np.linspace(1, len(path) - 2, sample_count, dtype=int)
        supported = 0
        checked = 0
        wide_samples = 0
        span = max(1, min(6, len(path) // 8))
        support_roi_start = int(round(component.shape[0] * self.config.roi_top_ratio))
        for index in indices:
            before_x, before_y = path[max(0, index - span)]
            after_x, after_y = path[min(len(path) - 1, index + span)]
            tangent_x, tangent_y = after_x - before_x, after_y - before_y
            norm = float(np.hypot(tangent_x, tangent_y))
            if norm < 1.0:
                continue
            normal_x, normal_y = -tangent_y / norm, tangent_x / norm
            x, y = path[index]
            # Only the course region below the configured ROI must be green
            # on both sides.  The far end of a real line can legitimately
            # leave the mat and point into the room, as in the live camera.
            if y < support_roi_start:
                continue
            probe = max(
                self.config.green_backbone_side_margin_pixels,
                int(round(float(distance[y, x]))) + self.config.green_backbone_side_margin_pixels,
            )
            wide_samples += int(float(distance[y, x]) > maximum_half_width)
            radius = self.config.green_backbone_green_probe_radius
            both_sides = (
                self._green_at(support_mask, x + normal_x * probe, y + normal_y * probe, radius)
                and self._green_at(support_mask, x - normal_x * probe, y - normal_y * probe, radius)
            )
            checked += 1
            supported += int(both_sides)
        ratio = supported / max(1, checked)
        wide_ratio = wide_samples / max(1, checked)
        return (
            checked >= minimum_samples
            and supported >= minimum_samples
            and ratio >= minimum_ratio
            and wide_ratio <= maximum_wide_ratio,
            ratio,
        )

    def _green_backbone_supported(self, component: np.ndarray) -> tuple[bool, float]:
        """Strict raw-green proof used whenever it is available."""
        return self._backbone_supported_by(
            component,
            self._latest_green_mask,
            minimum_ratio=self.config.green_backbone_min_supported_ratio,
            minimum_samples=self.config.green_backbone_min_supported_samples,
            maximum_half_width=self.config.green_backbone_max_tape_half_width,
            maximum_wide_ratio=self.config.green_backbone_max_wide_ratio,
        )

    def _course_backbone_supported(self, component: np.ndarray) -> tuple[bool, float]:
        """Fallback proof against the recovered green course, not raw HSV.

        `green_field` is deliberately hole-filled before white-tape masking,
        so it represents the carpet underneath tape and red markers.  This
        repairs close-range white tape that is real but lacks raw-green pixels
        immediately beside its skeleton.  It is not an unrestricted white
        fallback: the candidate must still be near-anchored, narrow enough,
        and surrounded by the same connected green course on both sides.
        """
        return self._backbone_supported_by(
            component,
            self._latest_green_field,
            minimum_ratio=self.config.course_backbone_min_supported_ratio,
            minimum_samples=self.config.course_backbone_min_supported_samples,
            maximum_half_width=self.config.course_backbone_max_tape_half_width,
            maximum_wide_ratio=self.config.course_backbone_max_wide_ratio,
        )

    def _select_route_component(self, mask: np.ndarray) -> np.ndarray:
        """Select an anchored route only after green-supported backbone proof."""
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
        best_label, best_score = 0, float("-inf")
        for label in range(1, count):
            x, y, component_width, component_height, area = map(int, stats[label])
            if component_height / max(1, height) < self.config.route_minimum_vertical_coverage:
                continue
            near_count = int(np.count_nonzero(near_labels == label))
            if near_count == 0:
                continue
            component = np.where(labels == label, 255, 0).astype(np.uint8)
            supported, support_ratio = self._green_backbone_supported(component)
            if not supported:
                supported, support_ratio = self._course_backbone_supported(component)
            if not supported:
                continue
            centre_distance = abs((x + component_width / 2) - width / 2) / max(1, width)
            score = near_count * 4.0 + area * .25 - centre_distance * area * .10 + support_ratio * area
            if score > best_score:
                best_label, best_score = label, score
        if best_label == 0:
            return np.zeros_like(constrained)
        return np.where(labels == best_label, 255, 0).astype(np.uint8)

    def _detect_red_band_marker(self, frame: np.ndarray, route_center_x: float | None) -> tuple[bool, int | None, int | None]:
        """Recover physical red layers despite left/right perspective skew.

        A layer can be two separated fragments around the white stem or a
        single intact strip.  Fragments are clustered by a generous but
        bounded Y interval, then exposed as ordered layers so higher-level
        motion code can distinguish ``two layers visible`` from ``far layer
        only`` without assuming the two halves share exactly one image row.
        """
        height, width = frame.shape[:2]
        config = self.config
        blue, green, red_channel = cv2.split(frame)
        red_excess = red_channel.astype(np.int16) - np.maximum(green, blue).astype(np.int16)
        red = np.where(
            (red_channel >= config.red_channel_min) & (red_excess >= config.red_excess_min),
            255,
            0,
        ).astype(np.uint8)
        course_allowed = self._latest_course_allowed
        if course_allowed is None or course_allowed.shape != red.shape:
            red = np.zeros_like(red)
        else:
            red = cv2.bitwise_and(red, course_allowed)
        cleanup = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, cleanup)
        self._latest_red_marker_mask = red
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
        fragments.sort(key=lambda item: item[6])
        tolerance = height * config.red_group_y_tolerance_ratio
        raw_groups: list[list[tuple[int, int, int, int, int, float, float]]] = []
        for fragment in fragments:
            if not raw_groups or fragment[6] - raw_groups[-1][-1][6] > tolerance:
                raw_groups.append([fragment])
            else:
                raw_groups[-1].append(fragment)

        layers: list[RedBandLayer] = []
        expected_x = width / 2.0 if route_center_x is None else route_center_x
        best: tuple[float, int, int] | None = None
        for group in raw_groups:
            left = min(item[0] for item in group)
            right = max(item[0] + item[2] - 1 for item in group)
            span = right - left + 1
            centre = (left + right) / 2.0
            total_area = sum(item[4] for item in group)
            marker_y = int(round(sum(item[4] * item[6] for item in group) / total_area))
            bottom_y = max(item[1] + item[3] - 1 for item in group)
            y_spread = int(round(max(item[6] for item in group) - min(item[6] for item in group)))
            layers.append(RedBandLayer(marker_y, bottom_y, span, len(group), y_spread))
            # A single wide strip is a valid far calibration layer.  Narrow
            # singleton noise is not.  Near split fragments remain valid even
            # when fish-eye projection offsets their centroids in Y.
            if span < width * config.red_min_span_ratio or abs(centre - expected_x) > width * config.red_centre_tolerance_ratio:
                continue
            score = span + min(total_area / max(1, height), width * .20)
            if best is None or score > best[0]:
                best = (score, marker_y, span)
        self._latest_red_band_layers = tuple(sorted(layers, key=lambda layer: layer.y))
        return (best is not None, None if best is None else best[1], None if best is None else best[2])

    def analyze(self, frame: np.ndarray):
        result = super().analyze(frame)
        evidence = result.evidence
        # A near transverse bar is intentionally not accepted as a steering
        # route.  It is nevertheless strong endpoint evidence even when it
        # fills the near view so completely that no narrow longitudinal
        # component can pass the route selector.  Analyse it independently
        # from the permissive candidate mask; it can only mark a bar, never
        # produce a left/right steering centre.
        if not evidence.endpoint_detected and self._latest_tape_candidate_mask is not None:
            bar_y, bar_width = self._find_wide_bar(self._latest_tape_candidate_mask, evidence.normal_tape_width)
            if bar_y is not None and bar_y >= frame.shape[0] * self.config.endpoint_min_y_ratio:
                evidence = replace(evidence, endpoint_detected=True, endpoint_y=bar_y, endpoint_width=bar_width)
        if not evidence.junction_detected and self._latest_tape_candidate_mask is not None:
            candidate_skeleton = self._skeletonize(self._latest_tape_candidate_mask)
            junction, junction_y, arms = self._detect_junction(candidate_skeleton)
            if junction and junction_y is not None and junction_y < frame.shape[0] * self.config.early_junction_max_y_ratio:
                evidence = replace(evidence, junction_detected=True, junction_y=junction_y, junction_arm_count=arms)
        detected, marker_y, marker_span = self._detect_red_band_marker(frame, evidence.line_center_x)
        if not detected or marker_y is None:
            return result.__class__(evidence, result.component_mask)
        # Red only pre-authorizes the white T. White endpoint confirmation and
        # stem loss are still required before braking or pivoting.
        junction_y = max(value for value in (evidence.junction_y, marker_y) if value is not None)
        red_evidence = replace(evidence, junction_detected=True, junction_y=junction_y, red_marker_detected=True, red_marker_y=marker_y, red_marker_span=marker_span)
        return result.__class__(red_evidence, result.component_mask)
