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
DEFAULT_EXPOSURE = {"auto": True, "ev": 0.0, "shutter_denominator": 200}
LOW_LATENCY_SIZE = (640, 480)
STREAM_PROFILES = {
    # Keep the camera running at the selected sensor mode, but constrain the
    # image which leaves the Pi.  MJPEG browsers always receive the newest
    # encoded frame, so no extra buffering is introduced by these profiles.
    "low_latency": {"label": "低延迟 · 640×480 · JPEG 60", "max_width": 640, "size": LOW_LATENCY_SIZE, "quality": 60},
    "balanced": {"label": "平衡 · 最宽 1230 px · JPEG 70", "max_width": 1230, "quality": 70},
    "source": {"label": "原始尺寸 · JPEG 70", "max_width": None, "quality": 70},
}
DEFAULT_STREAM_PROFILE = "low_latency"
DEFAULT_HIGHRES_FPS = 2.0
MIN_HIGHRES_FPS = 1.0
MAX_HIGHRES_FPS = 30.0
HIGHRES_JPEG_QUALITY = 75
HIGHRES_CACHE_MAX_AGE = 0.45
HIGHRES_PROFILES = {
    "source": {"label": "原始尺寸 · JPEG 75", "max_width": None},
    "medium_1640": {"label": "高清平衡 · 最宽 1640 px · JPEG 75", "max_width": 1640},
    "compact_1280": {"label": "高清轻量 · 最宽 1280 px · JPEG 75", "max_width": 1280},
}
DEFAULT_HIGHRES_PROFILE = "source"
# The CSI module's outer image area is visibly magenta in the supplied flat
# wall reference.  RGB888 arrays arrive in OpenCV BGR order, hence B/G/R.
DEFAULT_COLOR_CORRECTION = {"enabled": True, "strength": 1.0}
EDGE_BGR_GAINS = (0.92, 1.06, 0.78)


class CameraStreamer:
    transport = "mjpeg"

    def __init__(
        self,
        width: int | None = None,
        height: int | None = None,
        quality: int = 70,
        fps: float | None = None,
        mode_key: str | None = None,
        config_path: Path | None = None,
    ) -> None:
        self.config_path = Path(config_path or Path(__file__).with_name("camera_config.json")).expanduser()
        saved = self._load_saved_settings()
        saved_mode = saved["mode"]
        self.mode_key = mode_key if mode_key in CAMERA_MODES else saved_mode
        if self.mode_key not in CAMERA_MODES:
            self.mode_key = DEFAULT_CAMERA_MODE
        mode = CAMERA_MODES[self.mode_key]
        self.width = int(width if width is not None else mode["width"])
        self.height = int(height if height is not None else mode["height"])
        self.quality = int(quality)
        self.sensor_fps = float(mode["sensor_fps"])
        self.fps = max(1.0, float(fps if fps is not None else mode["stream_fps"]))
        self.auto_exposure = bool(saved["exposure"]["auto"])
        self.exposure_ev = float(saved["exposure"]["ev"])
        self.shutter_denominator = int(saved["exposure"]["shutter_denominator"])
        self.stream_profile_key = saved["stream_profile"]
        self.highres_profile_key = saved["highres_profile"]
        self.highres_fps = float(saved["highres_fps"])
        self.color_correction_enabled = bool(saved["color_correction"]["enabled"])
        self.color_correction_strength = float(saved["color_correction"]["strength"])
        self._color_gain_maps: dict[tuple[int, int, float], np.ndarray] = {}
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._highres_jpeg: bytes | None = None
        self._highres_sequence = 0
        self._next_highres_at = 0.0
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
        self._highres_events = deque()
        self._highres_stream_events = deque()
        self._highres_encode_time_events = deque()
        self._last_frame_at = 0.0
        self._last_jpeg_bytes = 0
        self._last_encode_ms = 0.0
        self._active_clients = 0
        self._last_highres_at = 0.0
        self._last_highres_jpeg_bytes = 0
        self._last_highres_encode_ms = 0.0
        self._highres_active_clients = 0

    def _load_saved_settings(self) -> dict:
        defaults = {
            "mode": DEFAULT_CAMERA_MODE,
            "exposure": dict(DEFAULT_EXPOSURE),
            "stream_profile": DEFAULT_STREAM_PROFILE,
            "highres_profile": DEFAULT_HIGHRES_PROFILE,
            "highres_fps": DEFAULT_HIGHRES_FPS,
            "color_correction": dict(DEFAULT_COLOR_CORRECTION),
        }
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
            if not isinstance(data, dict): return defaults
            mode = data.get("mode")
            if mode in CAMERA_MODES: defaults["mode"] = mode
            stream_profile = data.get("stream_profile")
            if stream_profile in STREAM_PROFILES: defaults["stream_profile"] = stream_profile
            highres_profile = data.get("highres_profile")
            if highres_profile in HIGHRES_PROFILES: defaults["highres_profile"] = highres_profile
            defaults["highres_fps"] = max(MIN_HIGHRES_FPS, min(MAX_HIGHRES_FPS, float(data.get("highres_fps", defaults["highres_fps"]))))
            exposure = data.get("exposure", {})
            if isinstance(exposure, dict):
                defaults["exposure"]["auto"] = bool(exposure.get("auto", defaults["exposure"]["auto"]))
                defaults["exposure"]["ev"] = max(-8.0, min(8.0, float(exposure.get("ev", defaults["exposure"]["ev"]))))
                defaults["exposure"]["shutter_denominator"] = max(1, int(exposure.get("shutter_denominator", defaults["exposure"]["shutter_denominator"])))
            correction = data.get("color_correction", {})
            if isinstance(correction, dict):
                defaults["color_correction"]["enabled"] = bool(correction.get("enabled", defaults["color_correction"]["enabled"]))
                defaults["color_correction"]["strength"] = max(0.0, min(1.5, float(correction.get("strength", defaults["color_correction"]["strength"]))))
        except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError, ValueError):
            pass
        return defaults

    def _save_settings(self) -> None:
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.config_path.with_name(f".{self.config_path.name}.tmp")
        payload = {
            "mode": self.mode_key,
            "stream_profile": self.stream_profile_key,
            "highres_profile": self.highres_profile_key,
            "highres_fps": self.highres_fps,
            "color_correction": {
                "enabled": self.color_correction_enabled,
                "strength": self.color_correction_strength,
            },
            "exposure": {
                "auto": self.auto_exposure,
                "ev": self.exposure_ev,
                "shutter_denominator": self.shutter_denominator,
            },
        }
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.config_path)

    def _stream_dimensions(self) -> tuple[int, int]:
        profile = STREAM_PROFILES[self.stream_profile_key]
        fixed_size = profile.get("size")
        if fixed_size is not None and self.width > fixed_size[0]:
            return fixed_size
        max_width = profile["max_width"]
        if max_width is None or self.width <= max_width:
            return self.width, self.height
        stream_width = int(max_width)
        stream_height = max(1, round(self.height * stream_width / self.width))
        return stream_width, stream_height

    def _stream_profile_status(self) -> dict:
        profile = STREAM_PROFILES[self.stream_profile_key]
        stream_width, stream_height = self._stream_dimensions()
        return {
            "key": self.stream_profile_key,
            "label": profile["label"],
            "max_width": profile["max_width"],
            "quality": profile["quality"],
            "width": stream_width,
            "height": stream_height,
            "resolution": f"{stream_width}x{stream_height}",
        }

    def _highres_dimensions(self) -> tuple[int, int]:
        max_width = HIGHRES_PROFILES[self.highres_profile_key]["max_width"]
        if max_width is None or self.width <= max_width:
            return self.width, self.height
        return int(max_width), max(1, round(self.height * int(max_width) / self.width))

    def _highres_profile_status(self) -> dict:
        profile = HIGHRES_PROFILES[self.highres_profile_key]
        width, height = self._highres_dimensions()
        return {
            "key": self.highres_profile_key,
            "label": profile["label"],
            "max_width": profile["max_width"],
            "quality": HIGHRES_JPEG_QUALITY,
            "width": width,
            "height": height,
            "resolution": f"{width}x{height}",
            "target_fps": self.highres_fps,
        }

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
            self._highres_events.clear()
            self._highres_stream_events.clear()
            self._highres_encode_time_events.clear()
            self._last_frame_at = 0.0
            self._last_jpeg_bytes = 0
            self._last_encode_ms = 0.0
            self._last_highres_at = 0.0
            self._last_highres_jpeg_bytes = 0
            self._last_highres_encode_ms = 0.0
            self._next_highres_at = 0.0
        with self._condition:
            self._jpeg = None
            self._sequence = 0
            self._highres_jpeg = None
            self._highres_sequence = 0

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

    def _record_highres_encoded(self, size: int, encode_ms: float) -> None:
        now = time.monotonic()
        with self._metrics_lock:
            self._highres_events.append((now, size))
            self._highres_encode_time_events.append((now, encode_ms))
            self._last_highres_at = now
            self._last_highres_jpeg_bytes = size
            self._last_highres_encode_ms = encode_ms

    def _record_highres_stream(self, size: int) -> None:
        with self._metrics_lock:
            self._highres_stream_events.append((time.monotonic(), size))

    def _client_started(self) -> None:
        with self._metrics_lock:
            self._active_clients += 1

    def _client_stopped(self) -> None:
        with self._metrics_lock:
            self._active_clients = max(0, self._active_clients - 1)

    def _highres_client_started(self) -> None:
        with self._metrics_lock:
            self._highres_active_clients += 1

    def _highres_client_stopped(self) -> None:
        with self._metrics_lock:
            self._highres_active_clients = max(0, self._highres_active_clients - 1)

    def _highres_is_due(self, now: float) -> bool:
        with self._metrics_lock:
            if self._highres_active_clients <= 0 or now < self._next_highres_at:
                return False
            self._next_highres_at = now + (1.0 / self.highres_fps)
            return True

    @property
    def shutter_us(self) -> int:
        return max(1, int(round(1_000_000 / self.shutter_denominator)))

    def _shutter_denominator_limits(self) -> tuple[int, int]:
        # At 30 FPS, an exposure longer than a frame is not meaningful.  The
        # camera may impose tighter limits, which are queried when available.
        min_us, max_us = 100, int(round(1_000_000 / self.sensor_fps))
        try:
            minimum, maximum, _ = self._picam2.camera_controls["ExposureTime"]
            min_us = max(1, int(minimum))
            max_us = min(max_us, int(maximum))
        except (AttributeError, KeyError, TypeError, ValueError):
            pass
        min_denominator = max(1, (1_000_000 + max_us - 1) // max_us)
        max_denominator = max(min_denominator, 1_000_000 // min_us)
        return min_denominator, max_denominator

    def _apply_exposure_controls(self) -> None:
        if self.auto_exposure:
            self._picam2.set_controls({"AeEnable": True, "ExposureValue": self.exposure_ev})
        else:
            self._picam2.set_controls({"AeEnable": False, "ExposureTime": self.shutter_us})

    def set_exposure(self, payload: dict) -> dict:
        """Apply EV in auto mode, or lock a shutter expressed as 1/N seconds."""
        if not isinstance(payload, dict):
            raise ValueError("曝光参数格式错误")
        try:
            auto = payload.get("auto", self.auto_exposure)
            if not isinstance(auto, bool): raise ValueError
            ev = max(-8.0, min(8.0, float(payload.get("ev", self.exposure_ev))))
            requested_denominator = int(round(float(payload.get("shutter_denominator", self.shutter_denominator))))
        except (TypeError, ValueError):
            raise ValueError("EV 或快门值无效") from None
        with self._lifecycle_lock:
            with self._camera_lock:
                minimum, maximum = self._shutter_denominator_limits()
                denominator = max(minimum, min(maximum, requested_denominator))
                previous = (self.auto_exposure, self.exposure_ev, self.shutter_denominator)
                self.auto_exposure, self.exposure_ev, self.shutter_denominator = auto, ev, denominator
                try:
                    if self._picam2 is not None: self._apply_exposure_controls()
                except Exception as exc:
                    self.auto_exposure, self.exposure_ev, self.shutter_denominator = previous
                    try:
                        if self._picam2 is not None: self._apply_exposure_controls()
                    except Exception:
                        pass
                    raise RuntimeError(f"应用曝光设置失败：{exc}") from exc
            self._save_settings()
        return self.status_dict()

    def set_stream_profile(self, profile_key: str) -> dict:
        """Change only the browser transport image; do not restart the sensor."""
        if profile_key not in STREAM_PROFILES:
            raise ValueError("未知视频传输档位")
        with self._lifecycle_lock:
            self.stream_profile_key = profile_key
            self._save_settings()
        return self.status_dict()

    def set_highres_profile(self, profile_key: str) -> dict:
        """Resize only the high-resolution JPEG channel."""
        if profile_key not in HIGHRES_PROFILES:
            raise ValueError("未知高清图片档位")
        with self._lifecycle_lock:
            self.highres_profile_key = profile_key
            self._save_settings()
        return self.status_dict()

    def set_highres_fps(self, value: float) -> dict:
        try:
            fps = float(value)
        except (TypeError, ValueError):
            raise ValueError("高清图片帧率必须是数字") from None
        if not MIN_HIGHRES_FPS <= fps <= MAX_HIGHRES_FPS:
            raise ValueError(f"高清图片帧率范围为 {MIN_HIGHRES_FPS:g}–{MAX_HIGHRES_FPS:g} FPS")
        with self._lifecycle_lock:
            self.highres_fps = fps
            with self._metrics_lock:
                self._next_highres_at = 0.0
            self._save_settings()
        return self.status_dict()

    def set_color_correction(self, payload: dict) -> dict:
        """Set persisted radial edge colour correction for BGR preview/JPEG frames."""
        if not isinstance(payload, dict):
            raise ValueError("颜色校正参数格式错误")
        try:
            enabled = payload.get("enabled", self.color_correction_enabled)
            if not isinstance(enabled, bool):
                raise ValueError
            strength = max(0.0, min(1.5, float(payload.get("strength", self.color_correction_strength))))
        except (TypeError, ValueError):
            raise ValueError("颜色校正强度必须是 0 到 1.5 的数字") from None
        with self._lifecycle_lock:
            self.color_correction_enabled, self.color_correction_strength = enabled, strength
            self._color_gain_maps.clear()
            self._save_settings()
        return self.status_dict()

    def _radial_color_gain_map(self, width: int, height: int):
        """Return a cached BGR gain map whose centre is exactly unchanged."""
        import numpy as np
        key = (int(width), int(height), round(self.color_correction_strength, 3))
        cached = self._color_gain_maps.get(key)
        if cached is not None:
            return cached
        x = np.linspace(-1.0, 1.0, int(width), dtype=np.float32)
        y = np.linspace(-1.0, 1.0, int(height), dtype=np.float32)
        radius = np.minimum(1.0, np.sqrt(y[:, None] ** 2 + x[None, :] ** 2) / np.sqrt(2.0))
        falloff = radius ** 2
        edge = np.asarray(EDGE_BGR_GAINS, dtype=np.float32).reshape(1, 1, 3)
        gain = 1.0 + (edge - 1.0) * (falloff[..., None] * self.color_correction_strength)
        self._color_gain_maps[key] = gain.astype(np.float32)
        return self._color_gain_maps[key]

    def _correct_edge_color(self, frame):
        if not self.color_correction_enabled or self.color_correction_strength <= 0.0:
            return frame
        try:
            gain = self._radial_color_gain_map(frame.shape[1], frame.shape[0])
            return self._cv2.multiply(frame, gain, dtype=self._cv2.CV_8U)
        except ImportError:
            # Development PCs may run the control service without OpenCV's
            # NumPy dependency. Keep the camera usable and leave the frame raw.
            return frame

    def status_dict(self) -> dict:
        now = time.monotonic()
        with self._metrics_lock:
            encoded_frames, encoded_bytes, _ = self._window_stats(self._encoded_events, now)
            stream_frames, stream_bytes, _ = self._window_stats(self._stream_events, now)
            highres_frames, highres_bytes, _ = self._window_stats(self._highres_events, now)
            highres_stream_frames, highres_stream_bytes, _ = self._window_stats(self._highres_stream_events, now)
            while self._encode_time_events and self._encode_time_events[0][0] < now - 1.0:
                self._encode_time_events.popleft()
            average_encode_ms = (
                sum(item[1] for item in self._encode_time_events) / len(self._encode_time_events)
                if self._encode_time_events else 0.0
            )
            while self._highres_encode_time_events and self._highres_encode_time_events[0][0] < now - 1.0:
                self._highres_encode_time_events.popleft()
            highres_encode_ms_avg = (
                sum(item[1] for item in self._highres_encode_time_events) / len(self._highres_encode_time_events)
                if self._highres_encode_time_events else 0.0
            )
            last_frame_at, last_jpeg_bytes, last_encode_ms, active_clients = (
                self._last_frame_at, self._last_jpeg_bytes, self._last_encode_ms, self._active_clients
            )
            last_highres_at, last_highres_jpeg_bytes, last_highres_encode_ms, highres_active_clients = (
                self._last_highres_at, self._last_highres_jpeg_bytes,
                self._last_highres_encode_ms, self._highres_active_clients,
            )
        return {
            "online": self.online,
            "status": self.status,
            "error": self.error,
            "mode": self.mode_key,
            "mode_label": CAMERA_MODES[self.mode_key]["label"],
            "exposure": {
                "auto": self.auto_exposure,
                "ev": self.exposure_ev,
                "shutter_denominator": self.shutter_denominator,
                "shutter_us": self.shutter_us,
            },
            "color_correction": {
                "enabled": self.color_correction_enabled,
                "strength": self.color_correction_strength,
                "edge_bgr_gains": EDGE_BGR_GAINS,
            },
            "available_modes": [self._mode_status_for(key) for key in CAMERA_MODES],
            "stream_profile": self._stream_profile_status(),
            "highres_profile": self._highres_profile_status(),
            "available_stream_profiles": [
                {"key": key, "label": profile["label"], "max_width": profile["max_width"], "quality": profile["quality"]}
                for key, profile in STREAM_PROFILES.items()
            ],
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
            "highres": {
                "target_fps": self.highres_fps,
                "capture_fps": highres_frames,
                "stream_fps": highres_stream_frames,
                "jpeg_bytes": last_highres_jpeg_bytes,
                "kBps": highres_bytes / 1000.0,
                "kbps": highres_bytes * 8.0 / 1000.0,
                "stream_kBps": highres_stream_bytes / 1000.0,
                "stream_kbps": highres_stream_bytes * 8.0 / 1000.0,
                "encode_ms": last_highres_encode_ms,
                "encode_ms_avg": highres_encode_ms_avg,
                "frame_age_ms": (now - last_highres_at) * 1000.0 if last_highres_at else None,
                "active_clients": highres_active_clients,
            },
            "available_highres_profiles": [
                {"key": key, "label": profile["label"], "max_width": profile["max_width"], "quality": HIGHRES_JPEG_QUALITY}
                for key, profile in HIGHRES_PROFILES.items()
            ],
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
                # The ISP performs this downscale before Python receives the
                # preview frame, avoiding a main-resolution resize at 30 FPS.
                lores={"size": LOW_LATENCY_SIZE, "format": "RGB888"},
                controls={
                    "FrameDurationLimits": (frame_duration_us, frame_duration_us),
                    "AwbEnable": True,
                },
                buffer_count=2,
            )
        )
        # Exposure settings are reapplied after every resolution switch.
        # In auto mode we leave gain/exposure to AEC/AGC and adjust only EV.
        self._apply_exposure_controls()

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
            self._save_settings()
            return self.status_dict()

    def _capture(self) -> None:
        interval = 1 / self.fps
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                profile = STREAM_PROFILES[self.stream_profile_key]
                use_lores = self.stream_profile_key == "low_latency"
                highres_due = self._highres_is_due(started)
                main_frame = None
                with self._camera_lock:
                    if self._picam2 is None:
                        break
                    request = self._picam2.capture_request()
                    try:
                        frame = request.make_array("lores" if use_lores else "main")
                        if highres_due:
                            main_frame = request.make_array("main")
                    finally:
                        request.release()
                stream_width, stream_height = self._stream_dimensions()
                if (frame.shape[1], frame.shape[0]) != (stream_width, stream_height):
                    frame = self._cv2.resize(frame, (stream_width, stream_height), interpolation=self._cv2.INTER_AREA)
                frame = self._correct_edge_color(frame)
                ok, buffer = self._cv2.imencode(
                    ".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, int(profile["quality"])]
                )
                if ok:
                    jpeg = buffer.tobytes()
                    self._record_encoded(len(jpeg), (time.monotonic() - started) * 1000.0)
                    with self._condition:
                        self._jpeg, self._sequence = jpeg, self._sequence + 1
                        self._condition.notify_all()
                # JPEG encode the auxiliary high-resolution image only after
                # the low-latency preview frame is published.
                if highres_due and main_frame is not None:
                    self._publish_highres_frame(main_frame)
                self._stop.wait(max(0, interval - (time.monotonic() - started)))
        except Exception as exc:
            self.status, self.error = "采集错误", str(exc)
            with self._condition: self._condition.notify_all()

    def _publish_highres_frame(self, frame) -> bytes | None:
        started = time.monotonic()
        highres_width, highres_height = self._highres_dimensions()
        if (frame.shape[1], frame.shape[0]) != (highres_width, highres_height):
            frame = self._cv2.resize(frame, (highres_width, highres_height), interpolation=self._cv2.INTER_AREA)
        frame = self._correct_edge_color(frame)
        ok, buffer = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, HIGHRES_JPEG_QUALITY])
        if not ok:
            return None
        jpeg = buffer.tobytes()
        self._record_highres_encoded(len(jpeg), (time.monotonic() - started) * 1000.0)
        with self._condition:
            self._highres_jpeg, self._highres_sequence = jpeg, self._highres_sequence + 1
            self._condition.notify_all()
        return jpeg

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

    def latest_highres_jpeg(self) -> bytes | None:
        """Return a recent high-resolution JPEG without writing to disk."""
        with self._condition:
            cached = self._highres_jpeg
        with self._metrics_lock:
            cache_age = time.monotonic() - self._last_highres_at if self._last_highres_at else float("inf")
        if cached is not None and cache_age <= HIGHRES_CACHE_MAX_AGE:
            return cached
        try:
            with self._camera_lock:
                if self._picam2 is None:
                    return cached
                request = self._picam2.capture_request()
                try:
                    frame = request.make_array("main")
                finally:
                    request.release()
            return self._publish_highres_frame(frame) or cached
        except Exception:
            return cached

    def iter_highres_mjpeg(self) -> Iterator[bytes]:
        """Share one 2 FPS high-resolution JPEG encoder between all clients."""
        last = -1
        self._highres_client_started()
        try:
            while not self._stop.is_set():
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._highres_sequence != last or self.error or self._stop.is_set(), timeout=1
                    )
                    frame, last = self._highres_jpeg, self._highres_sequence
                if self.error or self._stop.is_set(): break
                if frame:
                    payload = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    self._record_highres_stream(len(payload))
                    yield payload
        finally:
            self._highres_client_stopped()

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
