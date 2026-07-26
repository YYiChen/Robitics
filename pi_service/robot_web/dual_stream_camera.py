"""One CSI camera, with low-latency WebRTC video and high-resolution JPEG frames."""
from __future__ import annotations

import json
import threading
import time
from collections import deque
from collections.abc import Iterator
from fractions import Fraction
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen


DEFAULT_HIGHRES_FPS = 2.0
MIN_HIGHRES_FPS = 1.0
MAX_HIGHRES_FPS = 30.0
HIGHRES_JPEG_QUALITY = 75
# When no browser is watching the high-resolution feed, do not spend CPU on
# JPEG encoding.  A latest-image API call can still request one on demand.
HIGHRES_CACHE_MAX_AGE = 0.45
HIGHRES_PROFILES = {
    "source": {"label": "原始尺寸 · JPEG 75", "max_width": None},
    "medium_1640": {"label": "高清平衡 · 最大 1640 px · JPEG 75", "max_width": 1640},
    "compact_1280": {"label": "高清轻量 · 最大 1280 px · JPEG 75", "max_width": 1280},
}


class DualStreamCamera:
    """Own the CSI camera once and fan out lores H.264 plus main JPEG frames."""

    transport = "webrtc"
    highres_available = True

    def __init__(
        self,
        *,
        video_width: int = 640,
        video_height: int = 480,
        video_fps: float = 30.0,
        video_bitrate: int = 1_500_000,
        webrtc_gop_frames: int = 8,
        highres_width: int = 1640,
        highres_height: int = 1232,
        webrtc_port: int = 8889,
        webrtc_path: str = "cam",
        udp_output: str = "udp://127.0.0.1:1234?pkt_size=1316",
        config_path: Path | None = None,
        config_store=None,
    ) -> None:
        self.video_width, self.video_height = int(video_width), int(video_height)
        self.video_fps, self.video_bitrate = float(video_fps), int(video_bitrate)
        # A short GOP bounds the time a new WebRTC consumer can wait for an
        # independently decodable frame.  Baseline profile also excludes
        # B-frames, which prevents encoder reordering delay.
        self.webrtc_gop_frames = max(1, int(webrtc_gop_frames))
        self.width, self.height = int(highres_width), int(highres_height)
        self.port, self.path = int(webrtc_port), str(webrtc_path).strip("/") or "cam"
        self.udp_output = str(udp_output)
        self.config_path = Path(config_path or Path(__file__).with_name("camera_config.json"))
        self.config_store = config_store if config_path is None else None
        self.highres_profile_key, self.highres_fps = self._load_highres_settings()
        self.status, self.error = "未启动", ""
        self._picam2 = self._cv2 = self._encoder = self._output = None
        self.encoder_name = ""
        self._camera_lock = threading.RLock()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._running = False
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._metrics_lock = threading.Lock()
        self._events: deque[tuple[float, int]] = deque()
        self._stream_events: deque[tuple[float, int]] = deque()
        self._encode_events: deque[tuple[float, float]] = deque()
        self._last_frame_at = 0.0
        self._last_jpeg_bytes = 0
        self._last_encode_ms = 0.0
        self._active_clients = 0
        self._highres_wakeup = threading.Event()

    @property
    def _probe_url(self) -> str:
        return f"http://127.0.0.1:{self.port}/{self.path}/"

    def _load_highres_settings(self) -> tuple[str, float]:
        profile_key, highres_fps = "medium_1640", DEFAULT_HIGHRES_FPS
        try:
            saved = (
                self.config_store.read_section("camera")
                if self.config_store is not None
                else json.loads(self.config_path.read_text(encoding="utf-8"))
            )
            profile = saved.get("highres_profile")
            if profile in HIGHRES_PROFILES:
                profile_key = profile
            highres_fps = max(MIN_HIGHRES_FPS, min(MAX_HIGHRES_FPS, float(saved.get("highres_fps", highres_fps))))
        except (FileNotFoundError, json.JSONDecodeError, OSError, AttributeError, TypeError, ValueError):
            pass
        return profile_key, highres_fps

    def _save_highres_settings(self) -> None:
        payload: dict = {}
        try:
            saved = (
                self.config_store.read_section("camera")
                if self.config_store is not None
                else json.loads(self.config_path.read_text(encoding="utf-8"))
            )
            if isinstance(saved, dict):
                payload = saved
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            pass
        payload["highres_profile"] = self.highres_profile_key
        payload["highres_fps"] = self.highres_fps
        if self.config_store is not None:
            self.config_store.write_section("camera", payload)
            return
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.config_path.with_name(f".{self.config_path.name}.tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.config_path)

    def _media_server_online(self) -> bool:
        try:
            with urlopen(self._probe_url, timeout=0.2) as response:
                return 200 <= response.status < 500
        except (OSError, URLError):
            return False

    @property
    def online(self) -> bool:
        return self._running and not self.error and self._media_server_online()

    def _highres_dimensions(self) -> tuple[int, int]:
        max_width = HIGHRES_PROFILES[self.highres_profile_key]["max_width"]
        if max_width is None or self.width <= max_width:
            return self.width, self.height
        return int(max_width), max(1, round(self.height * int(max_width) / self.width))

    def _highres_profile_status(self) -> dict:
        width, height = self._highres_dimensions()
        profile = HIGHRES_PROFILES[self.highres_profile_key]
        return {"key": self.highres_profile_key, "label": profile["label"], "max_width": profile["max_width"], "quality": HIGHRES_JPEG_QUALITY, "width": width, "height": height, "resolution": f"{width}x{height}", "target_fps": self.highres_fps}

    @staticmethod
    def _window_stats(events: deque[tuple[float, int]], now: float) -> tuple[int, int]:
        while events and events[0][0] < now - 1.0:
            events.popleft()
        return len(events), sum(item[1] for item in events)

    def _record_frame(self, size: int, encode_ms: float) -> None:
        now = time.monotonic()
        with self._metrics_lock:
            self._events.append((now, size))
            self._encode_events.append((now, encode_ms))
            self._last_frame_at, self._last_jpeg_bytes, self._last_encode_ms = now, size, encode_ms

    def _record_stream(self, size: int) -> None:
        with self._metrics_lock:
            self._stream_events.append((time.monotonic(), size))

    def start(self) -> None:
        if self._running:
            return
        try:
            import cv2
            from picamera2 import Picamera2
            try:
                from picamera2.encoders import LibavH264Encoder as Encoder
                self.encoder_name = "LibavH264Encoder"
            except ImportError:
                from picamera2.encoders import H264Encoder as Encoder
                self.encoder_name = "H264Encoder"
            from picamera2.outputs import FfmpegOutput
            self._cv2 = cv2
            self._picam2 = Picamera2()
            frame_duration_us = int(round(1_000_000 / self.video_fps))
            config = self._picam2.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                lores={"size": (self.video_width, self.video_height), "format": "YUV420"},
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us), "AwbEnable": True},
                # Keep only two camera buffers: at 30 FPS this avoids up to
                # roughly 67 ms of extra capture queueing versus four buffers.
                buffer_count=2,
            )
            self._picam2.configure(config)
            self._encoder = Encoder(
                bitrate=self.video_bitrate,
                repeat=True,
                iperiod=self.webrtc_gop_frames,
                # Picamera2's Libav encoder expects a Fraction, not a float.
                framerate=Fraction(str(self.video_fps)),
                profile="baseline",
            )
            # These are output-side FFmpeg options.  They avoid the default
            # MPEG-TS mux delay before MediaMTX forwards packets to WebRTC.
            self._output = FfmpegOutput(
                f"-flush_packets 1 -muxdelay 0 -muxpreload 0 -f mpegts {self.udp_output}"
            )
            self._picam2.start_recording(self._encoder, self._output, name="lores")
            self._stop.clear()
            self._running = True
            self.status, self.error = "H.264 / WebRTC 与高清 JPEG 启动中", ""
            self._thread = threading.Thread(target=self._capture_highres, name="dual-camera-highres", daemon=True)
            self._thread.start()
        except Exception as exc:
            self.status, self.error, self._running = "不可用", str(exc), False
            self.stop()

    def _highres_client_started(self) -> None:
        with self._metrics_lock:
            self._active_clients += 1
        # Do not make a newly opened preview wait for the next 0.5 s slot.
        self._highres_wakeup.set()

    def _highres_client_stopped(self) -> None:
        with self._metrics_lock:
            self._active_clients = max(0, self._active_clients - 1)

    def _highres_has_clients(self) -> bool:
        with self._metrics_lock:
            return self._active_clients > 0

    def _capture_highres_frame(self) -> bytes | None:
        started = time.monotonic()
        with self._camera_lock:
            if self._picam2 is None:
                return None
            frame = self._picam2.capture_array("main")
        width, height = self._highres_dimensions()
        if (width, height) != (self.width, self.height):
            frame = self._cv2.resize(frame, (width, height), interpolation=self._cv2.INTER_AREA)
        ok, buffer = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, HIGHRES_JPEG_QUALITY])
        if not ok:
            return None
        jpeg = buffer.tobytes()
        self._record_frame(len(jpeg), (time.monotonic() - started) * 1000.0)
        with self._condition:
            self._jpeg, self._sequence = jpeg, self._sequence + 1
            self._condition.notify_all()
        return jpeg

    def _capture_highres(self) -> None:
        try:
            while not self._stop.is_set():
                if not self._highres_has_clients():
                    self._highres_wakeup.wait(timeout=1.0)
                    self._highres_wakeup.clear()
                    continue
                started = time.monotonic()
                self._capture_highres_frame()
                interval = 1.0 / self.highres_fps
                self._stop.wait(max(0.0, interval - (time.monotonic() - started)))
        except Exception as exc:
            self.status, self.error = "高清 JPEG 采集错误", str(exc)
            with self._condition:
                self._condition.notify_all()

    def set_highres_profile(self, profile_key: str) -> dict:
        if profile_key not in HIGHRES_PROFILES:
            raise ValueError("未知高清图片档位")
        self.highres_profile_key = profile_key
        self._save_highres_settings()
        return self.status_dict()

    def set_highres_fps(self, value: float) -> dict:
        try:
            fps = float(value)
        except (TypeError, ValueError):
            raise ValueError("高清图片帧率必须是数字") from None
        if not MIN_HIGHRES_FPS <= fps <= MAX_HIGHRES_FPS:
            raise ValueError(f"高清图片帧率范围为 {MIN_HIGHRES_FPS:g}–{MAX_HIGHRES_FPS:g} FPS")
        self.highres_fps = fps
        self._save_highres_settings()
        self._highres_wakeup.set()
        return self.status_dict()

    def latest_highres_jpeg(self) -> bytes | None:
        with self._condition:
            cached = self._jpeg
        with self._metrics_lock:
            cache_age = time.monotonic() - self._last_frame_at if self._last_frame_at else float("inf")
        if cached is not None and cache_age <= HIGHRES_CACHE_MAX_AGE:
            return cached
        # The one-shot API remains available for DL and verification even when
        # the browser preview is deliberately switched off.
        return self._capture_highres_frame() or cached

    def latest_jpeg(self) -> bytes | None:
        """Return the newest JPEG available from the dual-stream camera."""
        with self._condition:
            return self._jpeg

    def iter_highres_mjpeg(self) -> Iterator[bytes]:
        last = -1
        self._highres_client_started()
        try:
            while not self._stop.is_set():
                with self._condition:
                    self._condition.wait_for(lambda: self._sequence != last or self.error or self._stop.is_set(), timeout=1)
                    frame, last = self._jpeg, self._sequence
                if self.error or self._stop.is_set():
                    break
                if frame:
                    payload = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    self._record_stream(len(payload))
                    yield payload
        finally:
            self._highres_client_stopped()

    def status_dict(self) -> dict:
        now = time.monotonic()
        with self._metrics_lock:
            frame_count, frame_bytes = self._window_stats(self._events, now)
            stream_count, stream_bytes = self._window_stats(self._stream_events, now)
            while self._encode_events and self._encode_events[0][0] < now - 1.0:
                self._encode_events.popleft()
            encode_average = sum(item[1] for item in self._encode_events) / len(self._encode_events) if self._encode_events else 0.0
            last_at, last_size, last_encode, clients = self._last_frame_at, self._last_jpeg_bytes, self._last_encode_ms, self._active_clients
        media_online = self._media_server_online()
        status = "H.264 / WebRTC 与高清 JPEG 运行中" if self._running and media_online and not self.error else (self.error or "等待 MediaMTX 或 H.264 输入流")
        return {
            "online": self._running and media_online and not self.error,
            "status": status,
            "error": self.error,
            "transport": self.transport,
            "highres_available": True,
            "mode": "h264_webrtc_dual",
            "mode_label": "H.264 / WebRTC + 高清 JPEG",
            "width": self.video_width,
            "height": self.video_height,
            "resolution": f"{self.video_width}x{self.video_height}",
            "target_fps": self.video_fps,
            "sensor_target_fps": self.video_fps,
            "capture_fps": self.video_fps,
            "stream_fps": self.video_fps,
            "jpeg_bytes": 0,
            "jpeg_kBps": 0.0,
            "jpeg_kbps": 0.0,
            "stream_kBps": 0.0,
            "stream_kbps": 0.0,
            "encode_ms": 0.0,
            "encode_ms_avg": 0.0,
            "frame_age_ms": None,
            "active_clients": None,
            "jpeg_quality": None,
            "stream_profile": {
                "key": "h264_webrtc",
                "label": f"H.264 · {self.video_bitrate / 1_000_000:.1f} Mbps · WebRTC",
                "resolution": f"{self.video_width}x{self.video_height}",
                "encoder": self.encoder_name,
                "gop_frames": self.webrtc_gop_frames,
                "keyframe_interval_ms": round(self.webrtc_gop_frames / self.video_fps * 1000),
                "profile": "baseline",
                "low_latency_mux": True,
                "camera_buffer_count": 2,
            },
            "highres_profile": self._highres_profile_status(),
            "highres": {"target_fps": self.highres_fps, "capture_fps": frame_count, "stream_fps": stream_count, "jpeg_bytes": last_size, "kBps": frame_bytes / 1000.0, "kbps": frame_bytes * 8.0 / 1000.0, "stream_kBps": stream_bytes / 1000.0, "stream_kbps": stream_bytes * 8.0 / 1000.0, "encode_ms": last_encode, "encode_ms_avg": encode_average, "frame_age_ms": (now - last_at) * 1000.0 if last_at else None, "active_clients": clients},
            "available_modes": [],
            "available_stream_profiles": [],
            "available_highres_profiles": [{"key": key, "label": profile["label"], "max_width": profile["max_width"], "quality": HIGHRES_JPEG_QUALITY} for key, profile in HIGHRES_PROFILES.items()],
            "webrtc_port": self.port,
            "webrtc_path": self.path,
            "rtsp_url_template": f"rtsp://{{host}}:8554/{self.path}",
        }

    def stop(self) -> None:
        self._stop.set()
        self._highres_wakeup.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._camera_lock:
            if self._picam2:
                try:
                    self._picam2.stop_recording()
                except Exception:
                    pass
                try:
                    self._picam2.stop()
                except Exception:
                    pass
        self._running = False
