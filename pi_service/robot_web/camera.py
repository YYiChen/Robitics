"""CSI camera capture shared by all browser clients."""
from __future__ import annotations

import threading
import time
from collections import deque
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
        self._metrics_lock = threading.Lock()
        self._encoded_events = deque()
        self._stream_events = deque()
        self._encode_time_events = deque()
        self._last_frame_at = 0.0
        self._last_jpeg_bytes = 0
        self._last_encode_ms = 0.0
        self._active_clients = 0

    @staticmethod
    def _window_stats(events: deque, now: float) -> tuple[int, int, float]:
        """Return (event count, byte total, elapsed window) for the last second."""
        cutoff = now - 1.0
        while events and events[0][0] < cutoff:
            events.popleft()
        if not events:
            return 0, 0, 1.0
        return len(events), sum(item[1] for item in events), 1.0

    def _record_encoded(self, size: int, encode_ms: float) -> None:
        now = time.monotonic()
        with self._metrics_lock:
            self._encoded_events.append((now, size))
            self._encode_time_events.append((now, encode_ms))
            self._last_frame_at = now
            self._last_jpeg_bytes = size
            self._last_encode_ms = encode_ms

    def _record_stream(self, size: int) -> None:
        with self._metrics_lock:
            self._stream_events.append((time.monotonic(), size))

    def _client_started(self) -> None:
        with self._metrics_lock:
            self._active_clients += 1

    def _client_stopped(self) -> None:
        with self._metrics_lock:
            self._active_clients = max(0, self._active_clients - 1)

    def status_dict(self) -> dict:
        now = time.monotonic()
        with self._metrics_lock:
            encoded_frames, encoded_bytes, _ = self._window_stats(self._encoded_events, now)
            stream_frames, stream_bytes, _ = self._window_stats(self._stream_events, now)
            while self._encode_time_events and self._encode_time_events[0][0] < now - 1.0:
                self._encode_time_events.popleft()
            average_encode_ms = (
                sum(item[1] for item in self._encode_time_events) / len(self._encode_time_events)
                if self._encode_time_events else 0.0
            )
            last_frame_at, last_jpeg_bytes, last_encode_ms, active_clients = (
                self._last_frame_at, self._last_jpeg_bytes, self._last_encode_ms, self._active_clients
            )
        return {
            "online": self.online,
            "status": self.status,
            "error": self.error,
            "width": self.width,
            "height": self.height,
            "resolution": f"{self.width}x{self.height}",
            "target_fps": self.fps,
            "capture_fps": encoded_frames,
            "stream_fps": stream_frames,
            "jpeg_bytes": last_jpeg_bytes,
            "jpeg_kBps": encoded_bytes / 1000.0,
            "jpeg_kbps": encoded_bytes * 8.0 / 1000.0,
            "stream_kBps": stream_bytes / 1000.0,
            "stream_kbps": stream_bytes * 8.0 / 1000.0,
            "encode_ms": last_encode_ms,
            "encode_ms_avg": average_encode_ms,
            "frame_age_ms": (now - last_frame_at) * 1000.0 if last_frame_at else None,
            "active_clients": active_clients,
            "jpeg_quality": self.quality,
        }

    @property
    def online(self) -> bool:
        return self.status == "运行中" and not self.error

    def start(self) -> None:
        try:
            import cv2
            from picamera2 import Picamera2
            self._cv2 = cv2
            self._picam2 = Picamera2()
            self._picam2.configure(
                self._picam2.create_video_configuration(
                    # Picamera2's RGB888 numpy array is laid out as [B,G,R],
                    # which is already OpenCV's native channel order.
                    main={"size": (self.width, self.height), "format": "RGB888"},
                    controls={"FrameDurationLimits": (33333, 33333), "AwbEnable": True},
                    buffer_count=4,
                )
            )
            # Apply controls after configure.  A 5 ms exposure reduces motion
            # blur while automatic analogue gain compensates for low light.
            self._picam2.set_controls({"ExposureTime": 5000, "AnalogueGain": 0.0})
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
                frame = self._picam2.capture_array()
                ok, buffer = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    jpeg = buffer.tobytes()
                    self._record_encoded(len(jpeg), (time.monotonic() - started) * 1000.0)
                    with self._condition:
                        self._jpeg, self._sequence = jpeg, self._sequence + 1
                        self._condition.notify_all()
                self._stop.wait(max(0, interval - (time.monotonic() - started)))
        except Exception as exc:
            self.status, self.error = "采集错误", str(exc)
            with self._condition: self._condition.notify_all()

    def iter_mjpeg(self) -> Iterator[bytes]:
        last = -1
        self._client_started()
        try:
            while not self._stop.is_set():
                with self._condition:
                    self._condition.wait_for(lambda: self._sequence != last or self.error or self._stop.is_set(), timeout=1)
                    frame, last = self._jpeg, self._sequence
                if self.error or self._stop.is_set(): break
                if frame:
                    payload = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    self._record_stream(len(payload))
                    yield payload
        finally:
            self._client_stopped()

    def stop(self) -> None:
        self._stop.set()
        if self._picam2:
            try: self._picam2.stop()
            except Exception: pass
