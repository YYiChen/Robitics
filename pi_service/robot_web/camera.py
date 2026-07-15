"""CSI camera capture shared by all browser clients."""
from __future__ import annotations

import threading
import time
from collections.abc import Iterator


class CameraStreamer:
    def __init__(self, width: int = 640, height: int = 480, quality: int = 80, fps: float = 30) -> None:
        self.width, self.height, self.quality, self.fps = width, height, quality, max(1.0, fps)
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._stop = threading.Event()
        self.status, self.error = "未启动", ""
        self._picam2 = self._cv2 = None

    @property
    def online(self) -> bool:
        return self.status == "运行中" and not self.error

    def start(self) -> None:
        try:
            import cv2
            from picamera2 import Picamera2
            self._cv2 = cv2
            self._picam2 = Picamera2()
            self._picam2.configure(self._picam2.create_video_configuration(main={"size": (self.width, self.height)}))
            self._picam2.start()
            self.status = "运行中"
            threading.Thread(target=self._capture, name="camera-capture", daemon=True).start()
        except Exception as exc:
            self.status, self.error = "不可用", str(exc)

    def _capture(self) -> None:
        interval = 1 / self.fps
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                frame = self._cv2.cvtColor(self._picam2.capture_array(), self._cv2.COLOR_RGB2BGR)
                ok, buffer = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    with self._condition:
                        self._jpeg, self._sequence = buffer.tobytes(), self._sequence + 1
                        self._condition.notify_all()
                self._stop.wait(max(0, interval - (time.monotonic() - started)))
        except Exception as exc:
            self.status, self.error = "采集错误", str(exc)
            with self._condition: self._condition.notify_all()

    def iter_mjpeg(self) -> Iterator[bytes]:
        last = -1
        while not self._stop.is_set():
            with self._condition:
                self._condition.wait_for(lambda: self._sequence != last or self.error or self._stop.is_set(), timeout=1)
                frame, last = self._jpeg, self._sequence
            if self.error or self._stop.is_set(): break
            if frame: yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    def stop(self) -> None:
        self._stop.set()
        if self._picam2:
            try: self._picam2.stop()
            except Exception: pass
