"""Crop YOLO-detected poker chips and normalize their top ellipse to a circle."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
from numpy.typing import NDArray
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = ROOT / "data" / "raw" / "round test"
DEFAULT_OUTPUT = (
    ROOT / "data" / "work" / "chips" / "2026-07-23-round-rectification"
)
DEFAULT_MODEL = (
    ROOT
    / "models"
    / "assets"
    / "chip_recognition"
    / "yolo11n-localization-hard-negative-v3"
    / "best.pt"
)
SUPPORTED_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


Image = NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class EllipseEvidence:
    center_xy: tuple[float, float]
    axes_wh: tuple[float, float]
    angle_degrees: float
    aspect_ratio: float
    contour_fill_ratio: float
    center_offset_ratio: float
    quality: float


@dataclass(frozen=True, slots=True)
class ChipResult:
    source: str
    detection_index: int
    detector_confidence: float
    bbox_xyxy: tuple[int, int, int, int]
    expanded_bbox_xyxy: tuple[int, int, int, int]
    accepted: bool
    rejection_reason: str | None
    ellipse: EllipseEvidence | None
    crop_path: str
    overlay_path: str
    mask_path: str
    rectified_path: str | None
    comparison_path: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--confidence", type=float, default=0.50)
    parser.add_argument("--nms-iou", type=float, default=0.45)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--device", default="0")
    parser.add_argument("--padding", type=float, default=0.14)
    parser.add_argument("--output-size", type=int, default=384)
    parser.add_argument("--minimum-quality", type=float, default=0.52)
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if not args.input.is_dir():
        raise SystemExit(f"input directory is missing: {args.input}")
    if not args.model.is_file():
        raise SystemExit(f"chip-localization model is missing: {args.model}")
    if not 0.0 < args.confidence <= 1.0:
        raise SystemExit("--confidence must be in (0, 1]")
    if not 0.0 < args.nms_iou <= 1.0:
        raise SystemExit("--nms-iou must be in (0, 1]")
    if not 0.0 <= args.padding <= 0.5:
        raise SystemExit("--padding must be in [0, 0.5]")
    if args.output_size < 128:
        raise SystemExit("--output-size must be at least 128")
    if not 0.0 < args.minimum_quality <= 1.0:
        raise SystemExit("--minimum-quality must be in (0, 1]")


def _input_images(directory: Path) -> list[Path]:
    paths = sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_SUFFIXES
    )
    if not paths:
        raise SystemExit(f"no supported images found in: {directory}")
    return paths


def _expand_bbox(
    bbox_xyxy: Sequence[float],
    image_shape: Sequence[int],
    padding: float,
) -> tuple[int, int, int, int]:
    height, width = image_shape[:2]
    x1, y1, x2, y2 = bbox_xyxy
    box_width = x2 - x1
    box_height = y2 - y1
    return (
        max(0, int(math.floor(x1 - box_width * padding))),
        max(0, int(math.floor(y1 - box_height * padding))),
        min(width, int(math.ceil(x2 + box_width * padding))),
        min(height, int(math.ceil(y2 + box_height * padding))),
    )


def _center_component(mask: Image) -> Image:
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return np.zeros_like(mask)
    height, width = mask.shape
    image_center = np.array([width / 2.0, height / 2.0])
    diagonal = math.hypot(width, height)
    best_label = 0
    best_score = -math.inf
    for label in range(1, count):
        area = float(stats[label, cv2.CC_STAT_AREA])
        if area < width * height * 0.03:
            continue
        distance = float(np.linalg.norm(centroids[label] - image_center)) / diagonal
        score = area / (width * height) - 0.75 * distance
        if score > best_score:
            best_score = score
            best_label = label
    if best_label == 0:
        return np.zeros_like(mask)
    component = np.where(labels == best_label, 255, 0).astype(np.uint8)
    return component


def _grabcut_chip_mask(crop: Image) -> Image:
    height, width = crop.shape[:2]
    mask = np.full((height, width), cv2.GC_PR_BGD, dtype=np.uint8)
    border = max(2, round(min(width, height) * 0.025))
    mask[:border, :] = cv2.GC_BGD
    mask[-border:, :] = cv2.GC_BGD
    mask[:, :border] = cv2.GC_BGD
    mask[:, -border:] = cv2.GC_BGD

    center = (width // 2, height // 2)
    probable_axes = (max(4, round(width * 0.39)), max(4, round(height * 0.35)))
    certain_axes = (max(3, round(width * 0.12)), max(3, round(height * 0.10)))
    cv2.ellipse(mask, center, probable_axes, 0, 0, 360, cv2.GC_PR_FGD, -1)
    cv2.ellipse(mask, center, certain_axes, 0, 0, 360, cv2.GC_FGD, -1)

    background_model = np.zeros((1, 65), dtype=np.float64)
    foreground_model = np.zeros((1, 65), dtype=np.float64)
    cv2.grabCut(
        crop,
        mask,
        None,
        background_model,
        foreground_model,
        5,
        cv2.GC_INIT_WITH_MASK,
    )
    binary = np.where(
        (mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0
    ).astype(np.uint8)
    kernel_size = max(3, round(min(width, height) * 0.018))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
    return _center_component(binary)


def _fit_top_ellipse(mask: Image) -> tuple[EllipseEvidence | None, NDArray[np.int32] | None]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, None
    contour = max(contours, key=cv2.contourArea)
    if len(contour) < 20:
        return None, contour

    # A convex hull suppresses small crown/edge-pattern notches while retaining
    # the projected outer rim. AMS is more stable than the direct fit on the
    # slightly flattened lower edge caused by the visible chip thickness.
    hull = cv2.convexHull(contour)
    if len(hull) < 5:
        return None, contour
    (center_x, center_y), (axis_width, axis_height), angle = cv2.fitEllipseAMS(hull)
    major = max(axis_width, axis_height)
    minor = min(axis_width, axis_height)
    if major <= 1.0 or minor <= 1.0:
        return None, contour

    height, width = mask.shape
    ellipse_area = math.pi * axis_width * axis_height / 4.0
    contour_area = float(cv2.contourArea(contour))
    fill_ratio = contour_area / ellipse_area if ellipse_area else 0.0
    center_offset = math.hypot(center_x - width / 2, center_y - height / 2)
    center_offset_ratio = center_offset / math.hypot(width, height)
    aspect_ratio = minor / major

    fill_score = max(0.0, 1.0 - abs(1.0 - fill_ratio) / 0.40)
    center_score = max(0.0, 1.0 - center_offset_ratio / 0.18)
    coverage = ellipse_area / (width * height)
    coverage_score = min(1.0, coverage / 0.35)
    aspect_score = min(1.0, aspect_ratio / 0.42)
    quality = (
        0.35 * fill_score
        + 0.25 * center_score
        + 0.25 * coverage_score
        + 0.15 * aspect_score
    )
    evidence = EllipseEvidence(
        center_xy=(round(center_x, 3), round(center_y, 3)),
        axes_wh=(round(axis_width, 3), round(axis_height, 3)),
        angle_degrees=round(angle, 3),
        aspect_ratio=round(aspect_ratio, 5),
        contour_fill_ratio=round(fill_ratio, 5),
        center_offset_ratio=round(center_offset_ratio, 5),
        quality=round(quality, 5),
    )
    return evidence, contour


def _derive_top_ellipse_from_inlay(
    crop: Image,
    preliminary: EllipseEvidence,
) -> tuple[EllipseEvidence | None, NDArray[np.int32] | None]:
    """Use the flat white centre label to remove the visible-side bias.

    GrabCut includes the chip thickness along the lower rim. The centre label is
    coplanar with the top face and is concentric with it on this fixed chip
    design, so its ellipse gives a cleaner perspective ratio and orientation.
    The preliminary outer fit contributes only the top-face diameter scale.
    """

    height, width = crop.shape[:2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 105), (179, 105, 255))

    allowed = np.zeros((height, width), dtype=np.uint8)
    center = tuple(int(round(value)) for value in preliminary.center_xy)
    allowed_axes = tuple(
        max(3, int(round(value * 0.36))) for value in preliminary.axes_wh
    )
    cv2.ellipse(
        allowed,
        center,
        allowed_axes,
        preliminary.angle_degrees,
        0,
        360,
        255,
        -1,
        cv2.LINE_AA,
    )
    white = cv2.bitwise_and(white, allowed)
    kernel_size = max(3, round(min(width, height) * 0.012))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel, iterations=2)
    white = _center_component(white)
    contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, None
    contour = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(contour)
    if len(hull) < 5:
        return None, contour
    (center_x, center_y), (inner_width, inner_height), angle = cv2.fitEllipseAMS(
        hull
    )
    inner_major = max(inner_width, inner_height)
    inner_minor = min(inner_width, inner_height)
    outer_major = max(preliminary.axes_wh)
    if inner_major <= 1.0 or inner_minor <= 1.0:
        return None, contour
    diameter_ratio = inner_major / outer_major
    center_delta = math.hypot(
        center_x - preliminary.center_xy[0], center_y - preliminary.center_xy[1]
    ) / math.hypot(width, height)
    if not 0.43 <= diameter_ratio <= 0.78 or center_delta > 0.10:
        return None, contour

    # Preserve the reliable outer width from the segmentation while taking
    # both perspective axes and the orientation from the coplanar centre label.
    # A small inward bias deliberately follows the top rim rather than the
    # lower edge of the visible side wall. It also prevents table pixels from
    # leaking into the normalized circular view.
    scale = 0.96 * outer_major / inner_major
    corrected_width = inner_width * scale
    corrected_height = inner_height * scale
    inner_area = math.pi * inner_width * inner_height / 4.0
    inner_fill = float(cv2.contourArea(contour)) / inner_area if inner_area else 0.0
    inner_score = max(0.0, 1.0 - abs(1.0 - inner_fill) / 0.35)
    ratio_score = max(0.0, 1.0 - abs(0.60 - diameter_ratio) / 0.20)
    quality = 0.55 * preliminary.quality + 0.30 * inner_score + 0.15 * ratio_score
    corrected_major = max(corrected_width, corrected_height)
    corrected_minor = min(corrected_width, corrected_height)
    evidence = EllipseEvidence(
        center_xy=(round(center_x, 3), round(center_y, 3)),
        axes_wh=(round(corrected_width, 3), round(corrected_height, 3)),
        angle_degrees=round(angle, 3),
        aspect_ratio=round(corrected_minor / corrected_major, 5),
        contour_fill_ratio=preliminary.contour_fill_ratio,
        center_offset_ratio=round(
            math.hypot(center_x - width / 2, center_y - height / 2)
            / math.hypot(width, height),
            5,
        ),
        quality=round(min(1.0, quality), 5),
    )
    return evidence, contour


def _ellipse_to_circle(
    crop: Image,
    ellipse: EllipseEvidence,
    output_size: int,
) -> tuple[Image, Image]:
    center_x, center_y = ellipse.center_xy
    axis_width, axis_height = ellipse.axes_wh
    theta = math.radians(ellipse.angle_degrees)
    width_axis = np.array([math.cos(theta), math.sin(theta)], dtype=np.float32)
    height_axis = np.array([-math.sin(theta), math.cos(theta)], dtype=np.float32)
    source_center = np.array([center_x, center_y], dtype=np.float32)
    source = np.float32(
        [
            source_center,
            source_center + width_axis * (axis_width / 2.0),
            source_center + height_axis * (axis_height / 2.0),
        ]
    )
    destination_center = np.array(
        [output_size / 2.0, output_size / 2.0], dtype=np.float32
    )
    radius = output_size * 0.43
    destination = np.float32(
        [
            destination_center,
            destination_center + np.array([radius, 0.0], dtype=np.float32),
            destination_center + np.array([0.0, radius], dtype=np.float32),
        ]
    )
    transform = cv2.getAffineTransform(source, destination)
    rectified = cv2.warpAffine(
        crop,
        transform,
        (output_size, output_size),
        flags=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(32, 32, 32),
    )
    circle_mask = np.zeros((output_size, output_size), dtype=np.uint8)
    cv2.circle(
        circle_mask,
        (output_size // 2, output_size // 2),
        round(radius),
        255,
        -1,
        cv2.LINE_AA,
    )
    masked = cv2.bitwise_and(rectified, rectified, mask=circle_mask)
    masked[circle_mask == 0] = (32, 32, 32)
    return rectified, masked


def _letterbox(image: Image, size: tuple[int, int]) -> Image:
    target_width, target_height = size
    height, width = image.shape[:2]
    scale = min(target_width / width, target_height / height)
    resized = cv2.resize(
        image,
        (max(1, round(width * scale)), max(1, round(height * scale))),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC,
    )
    canvas = np.full((target_height, target_width, 3), 32, dtype=np.uint8)
    y = (target_height - resized.shape[0]) // 2
    x = (target_width - resized.shape[1]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return canvas


def _comparison(
    crop: Image,
    overlay: Image,
    rectified: Image | None,
    accepted: bool,
    quality: float | None,
) -> Image:
    panel_size = (420, 420)
    panels = [_letterbox(crop, panel_size), _letterbox(overlay, panel_size)]
    panels.append(
        _letterbox(rectified, panel_size)
        if rectified is not None
        else np.full((panel_size[1], panel_size[0], 3), 32, dtype=np.uint8)
    )
    labels = ("YOLO crop", "fitted top ellipse", "circle-normalized")
    for panel, label in zip(panels, labels):
        cv2.rectangle(panel, (0, 0), (panel.shape[1], 38), (20, 20, 20), -1)
        cv2.putText(
            panel,
            label,
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )
    result = np.hstack(panels)
    status = "accepted" if accepted else "rejected"
    quality_text = "n/a" if quality is None else f"{quality:.3f}"
    cv2.putText(
        result,
        f"ellipse {status} | quality {quality_text}",
        (12, result.shape[0] - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (80, 230, 80) if accepted else (60, 80, 240),
        2,
        cv2.LINE_AA,
    )
    return result


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve())).replace("\\", "/")
    except ValueError:
        return str(path.resolve())


def _write_image(path: Path, image: Image) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write image: {path}")


def _prepare_output(output: Path) -> None:
    for child in ("crops", "masks", "overlays", "rectified", "comparisons"):
        (output / child).mkdir(parents=True, exist_ok=True)


def run(args: argparse.Namespace) -> list[ChipResult]:
    _validate_args(args)
    inputs = _input_images(args.input)
    _prepare_output(args.output)
    model = YOLO(str(args.model.resolve()))
    names = model.names
    normalized_names = (
        {int(key): str(value) for key, value in names.items()}
        if isinstance(names, dict)
        else {index: str(value) for index, value in enumerate(names)}
    )
    if set(normalized_names) != {0} or normalized_names[0].strip().lower() not in {
        "pokerchip",
        "poker_chip",
        "chip",
    }:
        raise SystemExit(f"unexpected detector class map: {normalized_names}")

    predictions = model.predict(
        [str(path) for path in inputs],
        conf=args.confidence,
        iou=args.nms_iou,
        imgsz=args.imgsz,
        device=args.device,
        verbose=False,
    )
    records: list[ChipResult] = []
    for source_path, prediction in zip(inputs, predictions):
        image = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read image: {source_path}")
        boxes = prediction.boxes.xyxy.detach().cpu().numpy()
        confidences = prediction.boxes.conf.detach().cpu().numpy()
        order = np.argsort(-confidences)
        for output_index, prediction_index in enumerate(order, start=1):
            confidence = float(confidences[prediction_index])
            raw_box = boxes[prediction_index]
            bbox = tuple(int(round(value)) for value in raw_box)
            expanded = _expand_bbox(raw_box, image.shape, args.padding)
            x1, y1, x2, y2 = expanded
            crop = np.ascontiguousarray(image[y1:y2, x1:x2])
            mask = _grabcut_chip_mask(crop)
            preliminary_ellipse, contour = _fit_top_ellipse(mask)
            reference_contour: NDArray[np.int32] | None = None
            ellipse = preliminary_ellipse
            if preliminary_ellipse is not None:
                inlay_ellipse, reference_contour = _derive_top_ellipse_from_inlay(
                    crop, preliminary_ellipse
                )
                if inlay_ellipse is not None:
                    ellipse = inlay_ellipse

            rejection_reason: str | None = None
            if ellipse is None:
                rejection_reason = "no_stable_ellipse"
            elif ellipse.aspect_ratio < 0.34:
                rejection_reason = "view_too_oblique"
            elif not 0.62 <= ellipse.contour_fill_ratio <= 1.28:
                rejection_reason = "ellipse_contour_mismatch"
            elif ellipse.center_offset_ratio > 0.18:
                rejection_reason = "ellipse_off_center"
            elif ellipse.quality < args.minimum_quality:
                rejection_reason = "low_ellipse_quality"
            accepted = rejection_reason is None

            overlay = crop.copy()
            if contour is not None:
                cv2.drawContours(overlay, [contour], -1, (0, 210, 255), 3)
            if reference_contour is not None:
                cv2.drawContours(overlay, [reference_contour], -1, (255, 180, 0), 2)
            if ellipse is not None:
                cv2.ellipse(
                    overlay,
                    (
                        ellipse.center_xy,
                        ellipse.axes_wh,
                        ellipse.angle_degrees,
                    ),
                    (40, 220, 40) if accepted else (40, 40, 230),
                    4,
                    cv2.LINE_AA,
                )

            rectified: Image | None = None
            if accepted and ellipse is not None:
                _, rectified = _ellipse_to_circle(crop, ellipse, args.output_size)

            base_name = f"{source_path.stem}_chip_{output_index:02d}"
            crop_path = args.output / "crops" / f"{base_name}.png"
            mask_path = args.output / "masks" / f"{base_name}.png"
            overlay_path = args.output / "overlays" / f"{base_name}.png"
            rectified_path = (
                args.output / "rectified" / f"{base_name}.png"
                if rectified is not None
                else None
            )
            comparison_path = args.output / "comparisons" / f"{base_name}.jpg"
            _write_image(crop_path, crop)
            _write_image(mask_path, mask)
            _write_image(overlay_path, overlay)
            if rectified_path is not None and rectified is not None:
                _write_image(rectified_path, rectified)
            _write_image(
                comparison_path,
                _comparison(
                    crop,
                    overlay,
                    rectified,
                    accepted,
                    ellipse.quality if ellipse is not None else None,
                ),
            )
            records.append(
                ChipResult(
                    source=_relative(source_path),
                    detection_index=output_index,
                    detector_confidence=round(confidence, 6),
                    bbox_xyxy=bbox,
                    expanded_bbox_xyxy=expanded,
                    accepted=accepted,
                    rejection_reason=rejection_reason,
                    ellipse=ellipse,
                    crop_path=_relative(crop_path),
                    overlay_path=_relative(overlay_path),
                    mask_path=_relative(mask_path),
                    rectified_path=(
                        _relative(rectified_path) if rectified_path is not None else None
                    ),
                    comparison_path=_relative(comparison_path),
                )
            )
    return records


def main() -> int:
    args = parse_args()
    records = run(args)
    report = {
        "schema_version": "1.0",
        "task": "chip_crop_and_top_face_round_rectification",
        "model": _relative(args.model),
        "input": _relative(args.input),
        "output": _relative(args.output),
        "parameters": {
            "confidence": args.confidence,
            "nms_iou": args.nms_iou,
            "imgsz": args.imgsz,
            "padding": args.padding,
            "output_size": args.output_size,
            "minimum_quality": args.minimum_quality,
        },
        "detections": len(records),
        "accepted": sum(record.accepted for record in records),
        "results": [asdict(record) for record in records],
    }
    report_path = args.output / "report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if records and all(record.accepted for record in records) else 2


if __name__ == "__main__":
    raise SystemExit(main())
