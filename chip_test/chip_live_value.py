"""Attach fixed-design denomination evidence to live YOLO chip boxes."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Sequence

import cv2
import numpy as np
from numpy.typing import NDArray

from chip_template_matcher import (
    ChipTemplateMatcher,
    ColorMatch,
    TemplateMatch,
)
from rectify_chip_images import (
    _derive_top_ellipse_from_inlay,
    _ellipse_to_circle,
    _expand_bbox,
    _fit_top_ellipse,
    _grabcut_chip_mask,
)


Image = NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class ChipValueObservation:
    bbox_xyxy: tuple[int, int, int, int]
    denomination: int | None
    accepted: bool
    score: float
    margin: float
    ellipse_quality: float | None
    ellipse_aspect_ratio: float | None
    ellipse_minor_axis_px: float | None
    latency_ms: float
    decision_reason: str | None
    rejection_reason: str | None
    track_id: int | None = None
    source_frame: int | None = None
    best_frame_quality: float | None = None
    raw_color_denomination: int | None = None
    raw_color_score: float = 0.0
    raw_color_margin: float = 0.0
    digit_denomination: int | None = None
    digit_score: float = 0.0
    digit_margin: float = 0.0


def _unknown(
    bbox_xyxy: tuple[int, int, int, int],
    reason: str,
    started_ns: int,
    ellipse_quality: float | None = None,
    ellipse_aspect_ratio: float | None = None,
    ellipse_minor_axis_px: float | None = None,
) -> ChipValueObservation:
    return ChipValueObservation(
        bbox_xyxy=bbox_xyxy,
        denomination=None,
        accepted=False,
        score=0.0,
        margin=0.0,
        ellipse_quality=ellipse_quality,
        ellipse_aspect_ratio=ellipse_aspect_ratio,
        ellipse_minor_axis_px=ellipse_minor_axis_px,
        latency_ms=round((time.perf_counter_ns() - started_ns) / 1_000_000, 3),
        decision_reason=None,
        rejection_reason=reason,
    )


def _five_ring_ratios(normalized_chip: Image) -> tuple[float, float]:
    """Measure warm-ring coverage that distinguishes this fixed 5-chip."""

    hsv = cv2.cvtColor(normalized_chip, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(normalized_chip, cv2.COLOR_BGR2LAB)
    height, width = normalized_chip.shape[:2]
    y_grid, x_grid = np.ogrid[:height, :width]
    radius = np.sqrt(
        (x_grid - width / 2.0) ** 2 + (y_grid - height / 2.0) ** 2
    ) / min(height, width)
    annulus = (radius >= 0.34) & (radius <= 0.48)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    warm_hsv = (
        ((hue <= 42) | (hue >= 168))
        & (saturation >= 42)
        & annulus
    )
    warm_lab = (lab[:, :, 1] >= 138) & (saturation >= 30) & annulus
    warm = warm_hsv | warm_lab
    strongly_red = (lab[:, :, 1] >= 150) & (saturation >= 45) & annulus
    pixels = max(1, int(np.count_nonzero(annulus)))
    warm_ratio = float(np.count_nonzero(warm)) / pixels
    red_ratio = float(np.count_nonzero(strongly_red)) / pixels
    return warm_ratio, red_ratio


def _raw_outer_ring_signature(
    crop: Image,
    ellipse,
) -> NDArray[np.float32]:
    """Measure the top-face outer ring before any perspective warp."""

    height, width = crop.shape[:2]
    outer = np.zeros((height, width), dtype=np.uint8)
    inner = np.zeros((height, width), dtype=np.uint8)
    center = tuple(int(round(value)) for value in ellipse.center_xy)
    half_axes = tuple(max(2, value / 2.0) for value in ellipse.axes_wh)
    outer_axes = tuple(max(2, int(round(value * 0.96))) for value in half_axes)
    inner_axes = tuple(max(1, int(round(value * 0.68))) for value in half_axes)
    cv2.ellipse(
        outer,
        center,
        outer_axes,
        ellipse.angle_degrees,
        0,
        360,
        255,
        -1,
        cv2.LINE_AA,
    )
    cv2.ellipse(
        inner,
        center,
        inner_axes,
        ellipse.angle_degrees,
        0,
        360,
        255,
        -1,
        cv2.LINE_AA,
    )
    annulus = (outer >= 128) & (inner < 128)
    if np.count_nonzero(annulus) < 48:
        raise ValueError("raw outer ring contains too few pixels")
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
    return np.concatenate(
        (
            np.mean(hsv[annulus], axis=0),
            np.std(hsv[annulus], axis=0),
            np.mean(lab[annulus], axis=0),
            np.std(lab[annulus], axis=0),
        )
    ).astype(np.float32)


def _fuse_raw_color_and_digit(
    color: ColorMatch,
    digit: TemplateMatch,
) -> tuple[int | None, bool, float, float, str | None, str | None]:
    """Fuse independent raw-ring and rectified-number evidence."""

    active_denominations = tuple(
        denomination
        for denomination in color.scores
        if denomination in digit.scores
    )
    if not active_denominations:
        return None, False, 0.0, 0.0, None, "no_shared_denomination_scope"
    fused_scores = {
        denomination: 0.72 * color.scores[denomination]
        + 0.28 * digit.scores[denomination]
        for denomination in active_denominations
    }
    ranking = sorted(fused_scores, key=fused_scores.get, reverse=True)
    best = ranking[0]
    best_score = fused_scores[best]
    margin = (
        best_score - fused_scores[ranking[1]]
        if len(ranking) > 1
        else best_score
    )

    if not color.accepted and not digit.accepted:
        return None, False, best_score, margin, None, "colour_and_digit_rejected"
    if color.accepted and digit.accepted:
        color_value = color.denomination
        digit_value = digit.denomination
        if color_value == digit_value:
            reason = "colour_digit_agree"
        elif {color_value, digit_value} == {1, 5}:
            if color.margin < 0.08:
                return (
                    None,
                    False,
                    best_score,
                    margin,
                    None,
                    "one_five_colour_ambiguous",
                )
            best = int(color_value)
            best_score = fused_scores[best]
            alternatives = [
                score
                for denomination, score in fused_scores.items()
                if denomination != best
            ]
            margin = best_score - max(alternatives) if alternatives else best_score
            reason = "raw_ring_one_five_override"
        elif digit.best_score >= 0.80 and digit.margin >= 0.50:
            best = int(digit_value)
            best_score = fused_scores[best]
            alternatives = [
                score
                for denomination, score in fused_scores.items()
                if denomination != best
            ]
            margin = best_score - max(alternatives) if alternatives else best_score
            reason = "strong_digit_conflict_override"
        elif best_score < 0.58 or margin < 0.14:
            return None, False, best_score, margin, None, "colour_digit_conflict"
        else:
            reason = "fused_colour_digit"
    elif color.accepted:
        if color.best_score < 0.56 or color.margin < 0.20:
            return None, False, best_score, margin, None, "digit_rejected"
        best = int(color.denomination)
        best_score = fused_scores[best]
        alternatives = [
            score
            for denomination, score in fused_scores.items()
            if denomination != best
        ]
        margin = best_score - max(alternatives) if alternatives else best_score
        reason = "strong_raw_ring_only"
    else:
        if digit.best_score < 0.72 or digit.margin < 0.30:
            return None, False, best_score, margin, None, "raw_colour_rejected"
        best = int(digit.denomination)
        best_score = fused_scores[best]
        alternatives = [
            score
            for denomination, score in fused_scores.items()
            if denomination != best
        ]
        margin = best_score - max(alternatives) if alternatives else best_score
        reason = "strong_digit_only"

    accepted = best_score >= 0.44 and margin >= 0.05
    return (
        best if accepted else None,
        accepted,
        best_score,
        margin,
        reason if accepted else None,
        None if accepted else "fused_threshold",
    )


def _resolve_one_five(
    match: TemplateMatch,
    normalized_chip: Image,
) -> tuple[int | None, bool, float, str | None, str | None]:
    """Use the fixed red/yellow ring to veto or correct 1/5 confusion."""

    if not match.accepted or match.denomination not in {1, 5}:
        return (
            match.denomination,
            match.accepted,
            match.best_score,
            None,
            None if match.accepted else "template_rejected",
        )
    warm_ratio, red_ratio = _five_ring_ratios(normalized_chip)
    strong_five_ring = warm_ratio >= 0.12
    strong_one_ring = warm_ratio <= 0.03
    if match.denomination == 1 and strong_five_ring:
        corrected_score = max(
            match.scores[5],
            min(0.99, 0.58 + (warm_ratio - 0.12) * 1.25 + red_ratio * 0.25),
        )
        return 5, True, round(corrected_score, 6), "five_ring_override", None
    if match.denomination == 1 and not strong_one_ring:
        return None, False, match.best_score, None, "one_five_colour_ambiguous"
    if match.denomination == 5 and not strong_five_ring:
        return None, False, match.best_score, None, "five_ring_conflict"
    decision_reason = (
        "five_ring_confirmed"
        if match.denomination == 5
        else "one_ring_confirmed"
    )
    return match.denomination, True, match.best_score, decision_reason, None


def recognize_chip_value(
    matcher: ChipTemplateMatcher,
    image: Image,
    bbox_xyxy: Sequence[int],
    *,
    padding: float = 0.14,
    normalized_size: int = 384,
    minimum_ellipse_quality: float = 0.52,
    minimum_minor_axis_px: float = 42.0,
    minimum_aspect_ratio: float = 0.38,
    maximum_rectification_size: int = 224,
) -> ChipValueObservation:
    """Rectify and classify one detected chip without mutating game state."""

    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        raise ValueError("image must be a non-empty BGR frame")
    if len(bbox_xyxy) != 4:
        raise ValueError("bbox_xyxy must contain four values")
    if not 0.0 <= padding <= 0.5:
        raise ValueError("padding must be in [0, 0.5]")
    if normalized_size < 128:
        raise ValueError("normalized_size must be at least 128")
    if not 0.0 < minimum_ellipse_quality <= 1.0:
        raise ValueError("minimum_ellipse_quality must be in (0, 1]")
    if minimum_minor_axis_px <= 0.0:
        raise ValueError("minimum_minor_axis_px must be positive")
    if not 0.0 < minimum_aspect_ratio <= 1.0:
        raise ValueError("minimum_aspect_ratio must be in (0, 1]")
    if maximum_rectification_size < 128:
        raise ValueError("maximum_rectification_size must be at least 128")

    started_ns = time.perf_counter_ns()
    bbox = tuple(int(value) for value in bbox_xyxy)
    x1, y1, x2, y2 = _expand_bbox(bbox, image.shape, padding)
    if x2 - x1 < 24 or y2 - y1 < 24:
        return _unknown(bbox, "crop_too_small", started_ns)
    crop = np.ascontiguousarray(image[y1:y2, x1:x2])
    rectification_scale = min(
        1.0,
        maximum_rectification_size / max(crop.shape[:2]),
    )
    if rectification_scale < 1.0:
        crop = cv2.resize(
            crop,
            (
                max(1, round(crop.shape[1] * rectification_scale)),
                max(1, round(crop.shape[0] * rectification_scale)),
            ),
            interpolation=cv2.INTER_AREA,
        )
    try:
        outer_mask = _grabcut_chip_mask(crop)
        preliminary, _ = _fit_top_ellipse(outer_mask)
        if preliminary is None:
            return _unknown(bbox, "no_outer_ellipse", started_ns)
        ellipse, _ = _derive_top_ellipse_from_inlay(crop, preliminary)
        if ellipse is None:
            return _unknown(bbox, "no_center_inlay_ellipse", started_ns)
        minor_axis_px = min(ellipse.axes_wh) / rectification_scale
        if minor_axis_px < minimum_minor_axis_px:
            return _unknown(
                bbox,
                "too_far",
                started_ns,
                ellipse.quality,
                ellipse.aspect_ratio,
                minor_axis_px,
            )
        if ellipse.aspect_ratio < minimum_aspect_ratio:
            return _unknown(
                bbox,
                "too_flat",
                started_ns,
                ellipse.quality,
                ellipse.aspect_ratio,
                minor_axis_px,
            )
        if ellipse.quality < minimum_ellipse_quality:
            return _unknown(
                bbox,
                "low_ellipse_quality",
                started_ns,
                ellipse.quality,
                ellipse.aspect_ratio,
                minor_axis_px,
            )
        raw_color_signature = _raw_outer_ring_signature(crop, ellipse)
        color_match = matcher.match_color_signature(raw_color_signature)
        _, normalized = _ellipse_to_circle(crop, ellipse, normalized_size)
        digit_match = matcher.match_digit_shape(normalized)
    except (cv2.error, ValueError) as exc:
        return _unknown(
            bbox,
            f"rectification_error:{type(exc).__name__}",
            started_ns,
        )

    (
        denomination,
        accepted,
        score,
        margin,
        decision_reason,
        rejection_reason,
    ) = _fuse_raw_color_and_digit(
        color_match,
        digit_match,
    )
    return ChipValueObservation(
        bbox_xyxy=bbox,
        denomination=denomination,
        accepted=accepted,
        score=score,
        margin=round(margin, 6),
        ellipse_quality=ellipse.quality,
        ellipse_aspect_ratio=ellipse.aspect_ratio,
        ellipse_minor_axis_px=minor_axis_px,
        latency_ms=round((time.perf_counter_ns() - started_ns) / 1_000_000, 3),
        decision_reason=decision_reason,
        rejection_reason=rejection_reason,
        raw_color_denomination=color_match.denomination,
        raw_color_score=color_match.best_score,
        raw_color_margin=color_match.margin,
        digit_denomination=digit_match.denomination,
        digit_score=digit_match.best_score,
        digit_margin=digit_match.margin,
    )


def recognize_chip_values(
    matcher: ChipTemplateMatcher,
    image: Image,
    boxes_xyxy: Sequence[Sequence[int]],
    *,
    minimum_minor_axis_px: float = 42.0,
    minimum_aspect_ratio: float = 0.38,
) -> tuple[ChipValueObservation, ...]:
    """Classify a bounded batch of already-detected chips."""

    return tuple(
        recognize_chip_value(
            matcher,
            image,
            bbox_xyxy,
            minimum_minor_axis_px=minimum_minor_axis_px,
            minimum_aspect_ratio=minimum_aspect_ratio,
        )
        for bbox_xyxy in boxes_xyxy
    )
