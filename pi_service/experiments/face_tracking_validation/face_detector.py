"""MediaPipe face detector via Tasks API (0.10.x compatible)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import urllib.request

import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision


MODEL_NAME = "blaze_face_short_range.tflite"
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/latest/blaze_face_short_range.tflite"
MODEL_PATH = Path(__file__).resolve().parent / MODEL_NAME


def _ensure_model() -> str:
    if not MODEL_PATH.exists():
        print(f"Downloading MediaPipe face model ({MODEL_PATH.name})...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    return str(MODEL_PATH)


@dataclass(frozen=True)
class FaceDetectionResult:
    detected: bool
    center_x: float | None
    center_y: float | None
    offset_x: float | None
    offset_y: float | None
    box_width: int
    box_height: int
    score: float
    frame_width: int
    frame_height: int


class FaceDetector:
    """MediaPipe Tasks face detector, optimized for phone selfie distance."""

    def __init__(self, model_selection: int = 0, min_confidence: float = 0.5):
        model_path = _ensure_model()
        base_opts = mp_python.BaseOptions(model_asset_path=model_path)
        opts = vision.FaceDetectorOptions(
            base_options=base_opts,
            running_mode=vision.RunningMode.IMAGE,
            min_detection_confidence=min_confidence,
        )
        self._detector = vision.FaceDetector.create_from_options(opts)

    def detect(self, frame: np.ndarray) -> FaceDetectionResult:
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results = self._detector.detect(mp_image)

        if not results.detections:
            return FaceDetectionResult(
                detected=False, center_x=None, center_y=None,
                offset_x=None, offset_y=None, box_width=0, box_height=0,
                score=0.0, frame_width=w, frame_height=h,
            )

        best = max(results.detections, key=lambda d: d.categories[0].score)
        bbox = best.bounding_box
        x, y = bbox.origin_x, bbox.origin_y
        bw, bh = bbox.width, bbox.height
        center_x = float(x + bw / 2)
        center_y = float(y + bh / 2)

        return FaceDetectionResult(
            detected=True,
            center_x=center_x,
            center_y=center_y,
            offset_x=float(center_x - w / 2),
            offset_y=float(center_y - h / 2),
            box_width=bw,
            box_height=bh,
            score=float(best.categories[0].score),
            frame_width=w,
            frame_height=h,
        )

    def close(self) -> None:
        self._detector.close()
