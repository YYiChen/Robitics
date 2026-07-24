#!/usr/bin/env python3
"""Save a Raspberry Pi MJPEG stream directly to this Windows computer."""
from __future__ import annotations

import argparse
import shutil
import signal
import threading
import time
from datetime import datetime
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen


class MjpegRecorder:
    def __init__(self, stream_url: str, output_dir: Path, fps: float, min_free_gb: float) -> None:
        self.stream_url = stream_url
        self.output_dir = output_dir
        self.interval = 1.0 / max(0.1, fps)
        self.min_free_bytes = int(min_free_gb * 1024**3)
        self.stop_event = threading.Event()
        self.saved = 0
        self.last_error = ""

    @staticmethod
    def _frames(buffer: bytearray):
        """Yield complete JPEGs and keep an incomplete trailing frame."""
        frames: list[bytes] = []
        while True:
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                del buffer[:-1]
                break
            if start:
                del buffer[:start]
            end = buffer.find(b"\xff\xd9", 2)
            if end < 0:
                break
            frames.append(bytes(buffer[:end + 2]))
            del buffer[:end + 2]
        return frames

    def _save(self, jpeg: bytes) -> None:
        if shutil.disk_usage(self.output_dir).free < self.min_free_bytes:
            raise RuntimeError("可用磁盘空间低于安全阈值，已停止保存")
        now = datetime.now()
        day_dir = self.output_dir / now.strftime("%Y-%m-%d")
        day_dir.mkdir(parents=True, exist_ok=True)
        name = now.strftime("frame_%Y%m%d_%H%M%S_%f.jpg")
        (day_dir / name).write_bytes(jpeg)
        self.saved += 1

    def run(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        next_save = 0.0
        while not self.stop_event.is_set():
            try:
                request = Request(self.stream_url, headers={"User-Agent": "Robitics-Windows-Recorder/1.0"})
                print(f"连接视频流：{self.stream_url}")
                with urlopen(request, timeout=15) as response:
                    self.last_error = ""
                    buffer = bytearray()
                    while not self.stop_event.is_set():
                        chunk = response.read(8192)
                        if not chunk:
                            raise ConnectionError("视频流已关闭")
                        buffer.extend(chunk)
                        for jpeg in self._frames(buffer):
                            now = time.monotonic()
                            if now >= next_save:
                                self._save(jpeg)
                                next_save = now + self.interval
                                if self.saved % 25 == 0:
                                    print(f"已保存 {self.saved} 张 -> {self.output_dir}")
            except (OSError, URLError, ConnectionError, RuntimeError) as exc:
                self.last_error = str(exc)
                print(f"记录器：{exc}")
                if isinstance(exc, RuntimeError):
                    self.stop_event.set()
                else:
                    self.stop_event.wait(3.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="将树莓派 MJPEG 视频流按固定帧率保存为本机 JPG")
    parser.add_argument("--stream-url", required=True, help="例如 http://100.80.46.54:5000/video_feed")
    parser.add_argument("--output-dir", type=Path, default=Path(r"C:\Users\32126\Desktop\Robitics\Pic"))
    parser.add_argument("--fps", type=float, default=5.0, help="保存帧率，默认 5 FPS")
    parser.add_argument("--min-free-gb", type=float, default=5.0, help="最少保留的磁盘空间，默认 5 GB")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    recorder = MjpegRecorder(args.stream_url, args.output_dir, args.fps, args.min_free_gb)
    signal.signal(signal.SIGINT, lambda *_: recorder.stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: recorder.stop_event.set())
    print(f"开始本地保存：{args.output_dir}（{args.fps:g} FPS；Ctrl+C 停止）")
    recorder.run()
    print(f"已停止；本次保存 {recorder.saved} 张。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
