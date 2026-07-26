"""Training-free denomination matching for normalized fixed-design chip views."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from numpy.typing import NDArray


Image = NDArray[np.uint8]
DENOMINATIONS = (1, 5, 10, 20)
TEMPLATE_SIZE = 128
CENTER_FRACTION = 0.40
ROTATION_STEP_DEGREES = 10
SHAPE_WEIGHT = 0.10
SHAPE_TEMPERATURE = 0.03
COLOR_TEMPERATURE = 0.50


@dataclass(frozen=True, slots=True)
class TemplateMatch:
    denomination: int | None
    accepted: bool
    best_score: float
    margin: float
    scores: dict[int, float]
    source_id: str | None
    rotation_degrees: int | None
    latency_ms: float


@dataclass(frozen=True, slots=True)
class ColorMatch:
    denomination: int | None
    accepted: bool
    best_score: float
    margin: float
    scores: dict[int, float]
    distances: dict[int, float]


def center_number_view(
    normalized_chip: Image,
    fraction: float = CENTER_FRACTION,
) -> Image:
    """Return the central denomination area, excluding the circular ring text."""

    if normalized_chip.ndim not in (2, 3) or normalized_chip.size == 0:
        raise ValueError("normalized chip image must be a non-empty 2D/3D array")
    height, width = normalized_chip.shape[:2]
    side = max(32, int(round(min(height, width) * fraction)))
    side = min(side, height, width)
    center_x = width // 2
    center_y = height // 2
    x1 = max(0, center_x - side // 2)
    y1 = max(0, center_y - side // 2)
    return np.ascontiguousarray(normalized_chip[y1 : y1 + side, x1 : x1 + side])


def digit_mask(center_view: Image, output_size: int = TEMPLATE_SIZE) -> Image:
    """Extract dark central print as a normalized binary mask."""

    if center_view.ndim == 3:
        gray = cv2.cvtColor(center_view, cv2.COLOR_BGR2GRAY)
    else:
        gray = center_view.copy()
    gray = cv2.resize(gray, (output_size, output_size), interpolation=cv2.INTER_AREA)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4)).apply(gray)
    _, binary = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
    )

    # Only the central print is denomination evidence. This rejects the outer
    # LAS VEGAS / POKER CLUB text that confused the generic OCR pipeline.
    support = np.zeros_like(binary)
    cv2.circle(
        support,
        (output_size // 2, output_size // 2),
        round(output_size * 0.405),
        255,
        -1,
        cv2.LINE_AA,
    )
    binary = cv2.bitwise_and(binary, support)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    cleaned = np.zeros_like(binary)
    minimum_area = max(8, round(output_size * output_size * 0.0012))
    maximum_area = round(output_size * output_size * 0.22)
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if minimum_area <= area <= maximum_area:
            cleaned[labels == label] = 255
    cleaned = cv2.morphologyEx(
        cleaned,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2)),
    )
    return cleaned


def _rotate(mask: Image, degrees: int) -> Image:
    size = mask.shape[0]
    transform = cv2.getRotationMatrix2D((size / 2.0, size / 2.0), degrees, 1.0)
    return cv2.warpAffine(
        mask,
        transform,
        (size, size),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )


def _canonicalize(mask: Image, extent: int = 96) -> Image:
    """Remove small debris, then centre and scale the printed denomination."""

    if mask.ndim == 3 and mask.shape[2] == 1:
        mask = mask[:, :, 0]
    binary = np.where(mask > 127, 255, 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        return np.zeros((TEMPLATE_SIZE, TEMPLATE_SIZE), dtype=np.uint8)
    areas = stats[1:, cv2.CC_STAT_AREA]
    minimum_component_area = max(12, round(float(np.max(areas)) * 0.08))
    cleaned = np.zeros_like(binary)
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= minimum_component_area:
            cleaned[labels == label] = 255
    points = cv2.findNonZero(cleaned)
    if points is None:
        return np.zeros((TEMPLATE_SIZE, TEMPLATE_SIZE), dtype=np.uint8)
    x, y, width, height = cv2.boundingRect(points)
    cropped = cleaned[y : y + height, x : x + width]
    scale = min(extent / width, extent / height)
    resized_width = max(1, round(width * scale))
    resized_height = max(1, round(height * scale))
    resized = cv2.resize(
        cropped,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )
    canvas = np.zeros((TEMPLATE_SIZE, TEMPLATE_SIZE), dtype=np.uint8)
    x1 = (TEMPLATE_SIZE - resized_width) // 2
    y1 = (TEMPLATE_SIZE - resized_height) // 2
    canvas[y1 : y1 + resized_height, x1 : x1 + resized_width] = resized
    return canvas


def _shape_feature(mask: Image, cell_size: int = 8, bins: int = 9) -> NDArray[np.float32]:
    image = mask.astype(np.float32) / 255.0
    gradient_x = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = cv2.magnitude(gradient_x, gradient_y)
    angle = (
        cv2.phase(gradient_x, gradient_y, angleInDegrees=True) % 180.0
    ) / (180.0 / bins)
    histograms: list[float] = []
    for y in range(0, TEMPLATE_SIZE, cell_size):
        for x in range(0, TEMPLATE_SIZE, cell_size):
            cell_angle = angle[y : y + cell_size, x : x + cell_size].reshape(-1)
            cell_weight = magnitude[y : y + cell_size, x : x + cell_size].reshape(
                -1
            )
            lower = np.floor(cell_angle).astype(np.int16) % bins
            upper = (lower + 1) % bins
            fraction = cell_angle - np.floor(cell_angle)
            histogram = np.zeros(bins, dtype=np.float32)
            np.add.at(histogram, lower, cell_weight * (1.0 - fraction))
            np.add.at(histogram, upper, cell_weight * fraction)
            histograms.extend(histogram.tolist())
    vector = np.asarray(histograms, dtype=np.float32)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-8:
        raise ValueError("digit mask contains no usable foreground")
    return vector / norm


def color_signature(normalized_chip: Image) -> NDArray[np.float32]:
    """Summarize the fixed chip design's coloured ring under varied lighting."""

    if normalized_chip.ndim != 3 or normalized_chip.shape[2] != 3:
        raise ValueError("normalized chip must be a BGR image")
    height, width = normalized_chip.shape[:2]
    y_grid, x_grid = np.ogrid[:height, :width]
    radius = np.sqrt(
        (x_grid - width / 2.0) ** 2 + (y_grid - height / 2.0) ** 2
    ) / min(height, width)
    annulus = (radius >= 0.35) & (radius <= 0.48)
    hsv = cv2.cvtColor(normalized_chip, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(normalized_chip, cv2.COLOR_BGR2LAB)
    return np.concatenate(
        (
            np.mean(hsv[annulus], axis=0),
            np.std(hsv[annulus], axis=0),
            np.mean(lab[annulus], axis=0),
            np.std(lab[annulus], axis=0),
        )
    ).astype(np.float32)


def _softmax(values: NDArray[np.float32], temperature: float) -> NDArray[np.float32]:
    shifted = values / temperature
    shifted -= np.max(shifted)
    exponent = np.exp(shifted)
    return exponent / np.sum(exponent)


class ChipTemplateMatcher:
    """Load base masks once and compare queries against rotated variants."""

    def __init__(
        self,
        library_dir: Path,
        minimum_score: float = 0.58,
        minimum_margin: float = 0.035,
        rotation_step_degrees: int = ROTATION_STEP_DEGREES,
        allowed_denominations: tuple[int, ...] | None = None,
    ) -> None:
        if not 0.0 < minimum_score <= 1.0:
            raise ValueError("minimum_score must be in (0, 1]")
        if not 0.0 <= minimum_margin <= 1.0:
            raise ValueError("minimum_margin must be in [0, 1]")
        if rotation_step_degrees <= 0 or 360 % rotation_step_degrees:
            raise ValueError("rotation_step_degrees must divide 360")
        manifest_path = library_dir / "manifest.json"
        if not manifest_path.is_file():
            raise FileNotFoundError(f"template manifest is missing: {manifest_path}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest_denominations = tuple(int(value) for value in manifest["denominations"])
        if manifest_denominations != DENOMINATIONS:
            raise ValueError(f"unexpected denomination map: {manifest['denominations']}")
        selected_denominations = (
            DENOMINATIONS if allowed_denominations is None else tuple(allowed_denominations)
        )
        if (
            len(selected_denominations) < 2
            or len(set(selected_denominations)) != len(selected_denominations)
            or any(value not in manifest_denominations for value in selected_denominations)
        ):
            raise ValueError(
                "allowed denominations must contain at least two unique values "
                f"from {manifest_denominations}"
            )
        self._denominations = selected_denominations

        features: list[NDArray[np.float32]] = []
        labels: list[int] = []
        source_ids: list[str] = []
        rotations: list[int] = []
        color_signatures: list[NDArray[np.float32]] = []
        color_labels: list[int] = []
        for item in manifest["templates"]:
            denomination = int(item["denomination"])
            if denomination not in self._denominations:
                continue
            mask_path = library_dir / item["mask_file"]
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                raise FileNotFoundError(f"template mask is missing: {mask_path}")
            if mask.ndim == 3 and mask.shape[2] == 1:
                mask = mask[:, :, 0]
            if mask.shape != (TEMPLATE_SIZE, TEMPLATE_SIZE):
                raise ValueError(f"unexpected template shape {mask.shape}: {mask_path}")
            for degrees in range(0, 360, rotation_step_degrees):
                features.append(
                    _shape_feature(_canonicalize(_rotate(mask, degrees)))
                )
                labels.append(denomination)
                source_ids.append(str(item["template_id"]))
                rotations.append(degrees)
            signature = np.asarray(item.get("color_signature"), dtype=np.float32)
            if signature.shape != (12,):
                raise ValueError(
                    f"missing or invalid color signature: {item['template_id']}"
                )
            color_signatures.append(signature)
            color_labels.append(denomination)
        if not features:
            raise ValueError("template library is empty")
        self._features = np.stack(features)
        self._labels = np.asarray(labels, dtype=np.int16)
        self._source_ids = source_ids
        self._rotations = np.asarray(rotations, dtype=np.int16)
        self._color_signatures = np.stack(color_signatures)
        self._color_labels = np.asarray(color_labels, dtype=np.int16)
        self._color_scale = np.maximum(
            np.std(self._color_signatures, axis=0), 2.0
        )
        self._color_prototypes = {
            denomination: np.mean(
                self._color_signatures[self._color_labels == denomination], axis=0
            )
            for denomination in DENOMINATIONS
            if denomination in self._denominations
        }
        self.minimum_score = minimum_score
        self.minimum_margin = minimum_margin

    def match_color_signature(
        self,
        signature: NDArray[np.float32],
        *,
        minimum_score: float = 0.42,
        minimum_margin: float = 0.06,
    ) -> ColorMatch:
        """Compare a raw outer-ring signature with fixed-design prototypes."""

        query = np.asarray(signature, dtype=np.float32)
        if query.shape != (12,):
            raise ValueError("colour signature must have shape (12,)")
        color_distances = np.asarray(
            [
                np.mean(
                    (
                        (query - self._color_prototypes[denomination])
                        / self._color_scale
                    )
                    ** 2
                )
                for denomination in self._denominations
            ],
            dtype=np.float32,
        )
        probabilities = _softmax(-color_distances, COLOR_TEMPERATURE)
        scores = {
            denomination: float(probabilities[index])
            for index, denomination in enumerate(self._denominations)
        }
        distances = {
            denomination: float(color_distances[index])
            for index, denomination in enumerate(self._denominations)
        }
        ranking = sorted(scores, key=scores.get, reverse=True)
        best = ranking[0]
        best_score = scores[best]
        margin = best_score - scores[ranking[1]]
        accepted = best_score >= minimum_score and margin >= minimum_margin
        return ColorMatch(
            denomination=best if accepted else None,
            accepted=accepted,
            best_score=round(best_score, 6),
            margin=round(margin, 6),
            scores={key: round(value, 6) for key, value in scores.items()},
            distances={key: round(value, 6) for key, value in distances.items()},
        )

    def match_digit_shape(self, normalized_chip: Image) -> TemplateMatch:
        """Match only the central printed number after perspective correction."""

        started_ns = time.perf_counter_ns()
        center = center_number_view(normalized_chip)
        mask = digit_mask(center)
        try:
            query = _shape_feature(_canonicalize(mask))
        except ValueError:
            return TemplateMatch(
                denomination=None,
                accepted=False,
                best_score=0.0,
                margin=0.0,
                scores={value: 0.0 for value in self._denominations},
                source_id=None,
                rotation_degrees=None,
                latency_ms=(time.perf_counter_ns() - started_ns) / 1_000_000,
            )
        similarities = self._features @ query
        shape_scores = np.asarray(
            [
                np.max(similarities[self._labels == denomination])
                for denomination in self._denominations
            ],
            dtype=np.float32,
        )
        probabilities = _softmax(shape_scores, SHAPE_TEMPERATURE)
        scores = {
            denomination: float(probabilities[index])
            for index, denomination in enumerate(self._denominations)
        }
        ranking = sorted(scores, key=scores.get, reverse=True)
        best = ranking[0]
        best_score = scores[best]
        margin = best_score - scores[ranking[1]]
        candidates = np.flatnonzero(self._labels == best)
        best_index = int(candidates[np.argmax(similarities[candidates])])
        accepted = best_score >= 0.42 and margin >= 0.04
        return TemplateMatch(
            denomination=best if accepted else None,
            accepted=accepted,
            best_score=round(best_score, 6),
            margin=round(margin, 6),
            scores={key: round(value, 6) for key, value in scores.items()},
            source_id=self._source_ids[best_index],
            rotation_degrees=int(self._rotations[best_index]),
            latency_ms=round(
                (time.perf_counter_ns() - started_ns) / 1_000_000, 3
            ),
        )

    def match_normalized_chip(self, normalized_chip: Image) -> TemplateMatch:
        """Legacy fused matcher retained for offline comparison tooling."""

        started_ns = time.perf_counter_ns()
        center = center_number_view(normalized_chip)
        mask = digit_mask(center)
        try:
            query = _shape_feature(_canonicalize(mask))
        except ValueError:
            return TemplateMatch(
                denomination=None,
                accepted=False,
                best_score=0.0,
                margin=0.0,
                scores={value: 0.0 for value in self._denominations},
                source_id=None,
                rotation_degrees=None,
                latency_ms=(time.perf_counter_ns() - started_ns) / 1_000_000,
            )
        similarities = self._features @ query
        shape_scores = np.asarray(
            [
                np.max(similarities[self._labels == denomination])
                for denomination in self._denominations
            ],
            dtype=np.float32,
        )
        query_color = color_signature(normalized_chip)
        color_distances = np.asarray(
            [
                np.mean(
                    (
                        (query_color - self._color_prototypes[denomination])
                        / self._color_scale
                    )
                    ** 2
                )
                for denomination in self._denominations
            ],
            dtype=np.float32,
        )
        fused = SHAPE_WEIGHT * _softmax(
            shape_scores, SHAPE_TEMPERATURE
        ) + (1.0 - SHAPE_WEIGHT) * _softmax(
            -color_distances, COLOR_TEMPERATURE
        )
        scores = {
            denomination: float(fused[index])
            for index, denomination in enumerate(self._denominations)
        }
        ranking = sorted(scores, key=scores.get, reverse=True)
        best = ranking[0]
        best_score = scores[best]
        margin = best_score - scores[ranking[1]]
        candidates = np.flatnonzero(self._labels == best)
        best_index = int(candidates[np.argmax(similarities[candidates])])
        accepted = best_score >= self.minimum_score and margin >= self.minimum_margin
        return TemplateMatch(
            denomination=best if accepted else None,
            accepted=accepted,
            best_score=round(best_score, 6),
            margin=round(margin, 6),
            scores={key: round(value, 6) for key, value in scores.items()},
            source_id=self._source_ids[best_index],
            rotation_degrees=int(self._rotations[best_index]),
            latency_ms=round(
                (time.perf_counter_ns() - started_ns) / 1_000_000, 3
            ),
        )
