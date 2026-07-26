"""Small, dependency-free Haar face detector used only by the probe server."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class FaceResult:
    detected: bool
    center_x: float | None
    center_y: float | None
    width: int
    height: int
    area: int
    frame_width: int
    frame_height: int
    bbox: tuple[int, int, int, int] | None = None


class FaceDetector:
    """Haar frontal-face detector with no vehicle or route dependencies."""

    def __init__(self, cascade_path: str | None = None) -> None:
        path = cascade_path or (cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        self.cascade_path = path
        self._cascade = cv2.CascadeClassifier(path)
        if self._cascade.empty():
            raise RuntimeError(f"无法加载 Haar 人脸模型: {path}")

    def detect(self, frame: np.ndarray) -> FaceResult:
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        boxes = self._cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=4,
            minSize=(40, 40),
        )
        if len(boxes) == 0:
            return FaceResult(False, None, None, 0, 0, 0, width, height)
        x, y, box_width, box_height = max((tuple(map(int, box)) for box in boxes), key=lambda box: box[2] * box[3])
        return FaceResult(
            True,
            x + box_width / 2.0,
            y + box_height / 2.0,
            box_width,
            box_height,
            box_width * box_height,
            width,
            height,
            (x, y, box_width, box_height),
        )
