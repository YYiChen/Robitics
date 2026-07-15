"""CSI camera capture shared by all browser clients."""
from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Iterator
import json
from pathlib import Path


CAMERA_MODES = {
    "fast_1640": {
        "label": "1640×1232",
        "width": 1640,
        "height": 1232,
        "sensor_fps": 30.0,
        "stream_fps": 30.0,
    },
    "full_3280": {
        "label": "3280×2464",
        "width": 3280,
        "height": 2464,
        "sensor_fps": 30.0,
        "stream_fps": 30.0,
    },
}
DEFAULT_CAMERA_MODE = "fast_1640"


class CameraStreamer:
    def __init__(
        self,
        width: int | None = None,
        height: int | None = None,
        quality: int = 80,
        fps: float | None = None,
        mode_key: str | None = None,
        config_path: Path | None = None,
    ) -> None:
        self.config_path = Path(config_path or Path(__file__).with_name("camera_config.json")).expanduser()
        saved_mode = self._load_saved_mode()
        self.mode_key = mode_key if mode_key in CAMERA_MODES else saved_mode
        if self.mode_key not in CAMERA_MODES:
            self.mode_key = DEFAULT_CAMERA_MODE
        mode = CAMERA_MODES[self.mode_key]
        self.width = int(width if width is not None else mode["width"])
        self.height = int(height if height is not None else mode["height"])
        self.quality = int(quality)
        self.sensor_fps = float(mode["sensor_fps"])
        self.fps = max(1.0, float(fps if fps is not None else mode["stream_fps"]))
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._stop = threading.Event()
        self.status, self.error = "未启动", ""
        self._picam2 = self._cv2 = None
        self._camera_lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._capture_thread: threading.Thread | None = None
        self._running = False
        self._metrics_lock = threading.Lock()
        self._encoded_events = deque()
        self._stream_events = deque()
        self._encode_time_events = deque()
        self._last_frame_at = 0.0
        self._last_jpeg_bytes = 0
        self._last_encode_ms = 0.0
        self._active_clients = 0

    def _load_saved_mode(self) -> str:
        try:
            value = json.loads(self.config_path.read_text(encoding="utf-8")).get("mode")
            return value if value in CAMERA_MODES else DEFAULT_CAMERA_MODE
        except (FileNotFoundError, json.JSONDecodeError, OSError, AttributeError):
            return DEFAULT_CAMERA_MODE

    def _save_mode(self) -> None:
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.config_path.with_name(f".{self.config_path.name}.tmp")
        temporary.write_text(json.dumps({"mode": self.mode_key}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.config_path)

    def _mode_status(self) -> dict:
        mode = CAMERA_MODES[self.mode_key]
        return {
            "key": self.mode_key,
            "label": mode["label"],
            "width": mode["width"],
            "height": mode["height"],
            "sensor_fps": mode["sensor_fps"],
            "stream_fps": mode["stream_fps"],
        }

    def _reset_metrics(self) -> None:
        with self._metrics_lock:
            self._encoded_events.clear()
            self._stream_events.clear()
            self._encode_time_events.clear()
            self._last_frame_at = 0.0
            self._last_jpeg_bytes = 0
            self._last_encode_ms = 0.0
        with self._condition:
            self._jpeg = None
            self._sequence = 0

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
            "mode": self.mode_key,
            "mode_label": CAMERA_MODES[self.mode_key]["label"],
            "available_modes": [self._mode_status_for(key) for key in CAMERA_MODES],
            "width": self.width,
            "height": self.height,
            "resolution": f"{self.width}x{self.height}",
            "target_fps": self.fps,
            "sensor_target_fps": self.sensor_fps,
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

    @staticmethod
    def _mode_status_for(key: str) -> dict:
        mode = CAMERA_MODES[key]
        return {
            "key": key,
            "label": mode["label"],
            "width": mode["width"],
            "height": mode["height"],
            "sensor_fps": mode["sensor_fps"],
            "stream_fps": mode["stream_fps"],
        }

    @property
    def online(self) -> bool:
        return self.status == "运行中" and not self.error

    def _configure(self) -> None:
        frame_duration_us = int(round(1_000_000 / self.sensor_fps))
        self._picam2.configure(
            self._picam2.create_video_configuration(
                # Picamera2's RGB888 numpy array is laid out as [B,G,R],
                # which is already OpenCV's native channel order.
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls={
                    "FrameDurationLimits": (frame_duration_us, frame_duration_us),
                    "AwbEnable": True,
                },
                buffer_count=4,
            )
        )
        # Keep the existing motion-blur-oriented exposure baseline.  The
        # sensor readout rate is controlled independently by FrameDurationLimits.
        self._picam2.set_controls({"ExposureTime": 5000, "AnalogueGain": 0.0})

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._running: return
            try:
                import cv2
                from picamera2 import Picamera2
                self._cv2 = cv2
                self._picam2 = Picamera2()
                with self._camera_lock:
                    self._configure()
                    self._picam2.start()
                self._stop.clear()
                self._capture_thread = threading.Thread(target=self._capture, name="camera-capture", daemon=True)
                self._capture_thread.start()
                self._running = True
                self.status, self.error = "运行中", ""
            except Exception as exc:
                self.status, self.error = "不可用", str(exc)

    def set_mode(self, mode_key: str) -> dict:
        """Switch resolution while keeping the sensor frame duration at 30 FPS."""
        if mode_key not in CAMERA_MODES:
            raise ValueError("未知相机模式")
        with self._lifecycle_lock:
            if mode_key == self.mode_key and self._running and self.online:
                return self.status_dict()
            previous = (self.mode_key, self.width, self.height, self.sensor_fps, self.fps)
            self.status, self.error = "切换中", ""
            self._stop.set()
            thread = self._capture_thread
            if thread and thread is not threading.current_thread():
                thread.join(timeout=2.0)
            new_mode = CAMERA_MODES[mode_key]
            self.mode_key = mode_key
            self.width, self.height = new_mode["width"], new_mode["height"]
            self.sensor_fps, self.fps = new_mode["sensor_fps"], new_mode["stream_fps"]
            try:
                with self._camera_lock:
                    if self._picam2 is None:
                        raise RuntimeError("摄像头尚未启动")
                    try:
                        self._picam2.stop()
                    except Exception:
                        pass
                    self._configure()
                    self._picam2.start()
            except Exception as exc:
                # Try to restore the previous working mode before reporting the
                # failure, so a bad sensor mode cannot leave the stream down.
                self.mode_key, self.width, self.height, self.sensor_fps, self.fps = previous
                try:
                    with self._camera_lock:
                        self._configure()
                        self._picam2.start()
                    self._stop.clear()
                    self._capture_thread = threading.Thread(target=self._capture, name="camera-capture", daemon=True)
                    self._capture_thread.start()
                    self._running = True
                    self.status = "运行中"
                except Exception as restore_exc:
                    self._running = False
                    self.status = "不可用"
                    self.error = f"切换失败：{exc}；恢复失败：{restore_exc}"
                raise RuntimeError(f"相机模式切换失败：{exc}") from exc
            self._reset_metrics()
            self._stop.clear()
            self._capture_thread = threading.Thread(target=self._capture, name="camera-capture", daemon=True)
            self._capture_thread.start()
            self._running = True
            self.status, self.error = "运行中", ""
            self._save_mode()
            return self.status_dict()

    def _capture(self) -> None:
        interval = 1 / self.fps
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                with self._camera_lock:
                    if self._picam2 is None: break
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
        with self._lifecycle_lock:
            self._stop.set()
            thread = self._capture_thread
            if thread and thread is not threading.current_thread():
                thread.join(timeout=2.0)
            with self._camera_lock:
                if self._picam2:
                    try: self._picam2.stop()
                    except Exception: pass
            self._running = False
