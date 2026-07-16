"""Status adapter for the H.264/WebRTC video pipeline.

The CSI camera is owned by rpicam-vid in this mode, while MediaMTX distributes
that stream to the browser as WebRTC and to DL clients as RTSP.  Flask must not
open Picamera2 concurrently with that process.
"""
from __future__ import annotations

import time
from urllib.error import URLError
from urllib.request import urlopen


class WebRTCStreamer:
    transport = "webrtc"

    def __init__(self, width: int, height: int, fps: float, bitrate: int, port: int, path: str) -> None:
        self.width, self.height = int(width), int(height)
        self.fps, self.bitrate = float(fps), int(bitrate)
        self.port, self.path = int(port), str(path).strip("/") or "cam"
        self.status, self.error = "未启动", ""

    @property
    def _probe_url(self) -> str:
        return f"http://127.0.0.1:{self.port}/{self.path}/"

    def start(self) -> None:
        self.status, self.error = "等待 MediaMTX WebRTC 流", ""

    def stop(self) -> None:
        self.status = "已停止"

    def _media_server_online(self) -> bool:
        try:
            with urlopen(self._probe_url, timeout=0.2) as response:
                return 200 <= response.status < 500
        except (OSError, URLError):
            return False

    def status_dict(self) -> dict:
        online = self._media_server_online()
        status = "H.264 / WebRTC 运行中" if online else "等待 MediaMTX 或 H.264 输入流"
        return {
            "online": online,
            "status": status,
            "error": self.error,
            "transport": self.transport,
            "mode": "h264_webrtc",
            "mode_label": "H.264 / WebRTC 低延迟",
            "width": self.width,
            "height": self.height,
            "resolution": f"{self.width}x{self.height}",
            "target_fps": self.fps,
            "sensor_target_fps": self.fps,
            "capture_fps": 0.0,
            "stream_fps": 0.0,
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
                "label": f"H.264 · {self.bitrate / 1_000_000:.1f} Mbps · WebRTC",
                "resolution": f"{self.width}x{self.height}",
            },
            "available_modes": [],
            "available_stream_profiles": [],
            "webrtc_port": self.port,
            "webrtc_path": self.path,
            "rtsp_url_template": f"rtsp://{{host}}:8554/{self.path}",
            "started_at": time.time(),
        }
