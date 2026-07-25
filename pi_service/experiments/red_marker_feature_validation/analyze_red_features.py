"""Offline comparison of red-marker feature separability on one camera frame."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def _best_balanced_accuracy(feature: np.ndarray, positive: np.ndarray, region: np.ndarray) -> tuple[float, float]:
    negatives = region & ~positive
    thresholds = np.unique(np.percentile(feature[region], np.linspace(0, 100, 301)))
    best_score, best_threshold = -1.0, 0.0
    for threshold in thresholds:
        true_positive_rate = float(np.mean(feature[positive] >= threshold))
        true_negative_rate = float(np.mean(feature[negatives] < threshold))
        score = (true_positive_rate + true_negative_rate) / 2.0
        if score > best_score:
            best_score, best_threshold = score, float(threshold)
    return best_score, best_threshold


def analyze(image: np.ndarray) -> dict[str, object]:
    height, width = image.shape[:2]
    blue, green, red = cv2.split(image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Offline reference only: matches the existing calibrated red-band range.
    marker = (((hsv[:, :, 0] <= 12) | (hsv[:, :, 0] >= 165)) & (hsv[:, :, 1] >= 85) & (hsv[:, :, 2] >= 70))
    # Avoid unrelated red objects near the room edge; compare the lower/middle
    # course view where the T-marker is expected.
    region = np.zeros((height, width), dtype=bool)
    region[int(height * .45):int(height * .88), int(width * .18):int(width * .82)] = True
    marker &= region
    features = {
        "red_channel": red.astype(np.float32),
        "red_excess": red.astype(np.float32) - np.maximum(green, blue).astype(np.float32),
        "gray_minus_green": gray.astype(np.float32) - green.astype(np.float32),
    }
    negatives = region & ~marker
    report: dict[str, object] = {
        "marker_pixels": int(np.count_nonzero(marker)),
        "non_marker_pixels": int(np.count_nonzero(negatives)),
        "features": {},
    }
    for name, feature in features.items():
        score, threshold = _best_balanced_accuracy(feature, marker, region)
        positive_median = float(np.median(feature[marker]))
        negative_median = float(np.median(feature[negatives]))
        report["features"][name] = {
            "marker_quantiles_5_50_95": [round(float(v), 2) for v in np.percentile(feature[marker], (5, 50, 95))],
            "non_marker_quantiles_5_50_95": [round(float(v), 2) for v in np.percentile(feature[negatives], (5, 50, 95))],
            "median_gap": round(positive_median - negative_median, 2),
            "best_balanced_accuracy": round(score, 4),
            "best_threshold": round(threshold, 2),
        }
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"Cannot read image: {args.image}")
    print(json.dumps(analyze(image), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
