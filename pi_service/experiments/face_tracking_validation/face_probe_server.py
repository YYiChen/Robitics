"""Read the existing port-5000 MJPEG stream and expose Haar detections on 5058.

This is an observation-only experiment.  It has no robot-controller import,
no keyboard handler, and no HTTP endpoint that can move a motor.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
from urllib.request import urlopen

import cv2
import numpy as np
from flask import Flask, Response, jsonify

from face_detector import FaceDetector, FaceResult


HTML = """<!doctype html><meta charset=utf-8><title>Haar Face Probe</title>
<style>body{margin:0;background:#09111d;color:#dbeafe;font:16px system-ui;text-align:center}main{max-width:900px;margin:18px auto}img{width:100%;background:#000;border-radius:8px}.note{color:#9fb3c8}</style>
<main><h2>Haar 人脸灵敏度探索（仅预览，无电机）</h2><p id=s class=note>连接中…</p><img src=/video_feed><p class=note>测试时正面面对镜头，逐步后退；记录连续检出时的实际距离和框宽。</p></main>
<script>setInterval(async()=>{let x=await fetch('/status');let s=await x.json();document.querySelector('#s').textContent=`检测=${s.detected} | 近5秒检出率=${s.detect_rate_5s}% | 框宽=${s.width}px | 耗时=${s.processing_ms}ms | FPS=${s.source_fps}`},400)</script>"""


class FaceProbe:
    def __init__(self, source_url: str, log_path: Path) -> None:
        self.source_url, self.log_path = source_url, log_path
        self.detector = FaceDetector()
        self._lock = threading.Lock()
        self._latest: bytes | None = None
        self._status = {"detected": False, "width": 0, "area": 0, "processing_ms": None, "source_fps": 0.0, "detect_rate_5s": 0.0, "frames": 0, "error": "starting"}
        self._stop = threading.Event()
        self._detections: deque[tuple[float, bool]] = deque()

    def start(self) -> None:
        threading.Thread(target=self._run, daemon=True, name="haar-face-probe").start()

    def snapshot(self) -> tuple[bytes | None, dict]:
        with self._lock:
            return self._latest, dict(self._status)

    def _record(self, frame_index: int, result: FaceResult, processing_ms: float, source_fps: float) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"time": datetime.now(timezone.utc).isoformat(), "frame": frame_index, "processing_ms": round(processing_ms, 2), "source_fps": round(source_fps, 2), **asdict(result)}
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def _run(self) -> None:
        buffer, frame_index, last_at = bytearray(), 0, time.monotonic()
        try:
            with urlopen(self.source_url, timeout=10) as response:
                while not self._stop.is_set():
                    chunk = response.read(8192)
                    if not chunk:
                        raise RuntimeError("5000 video_feed 已结束")
                    buffer.extend(chunk)
                    start, end = buffer.find(b"\xff\xd8"), buffer.find(b"\xff\xd9")
                    if start < 0 or end < start:
                        if len(buffer) > 2_000_000:
                            buffer.clear()
                        continue
                    jpeg = bytes(buffer[start:end + 2])
                    del buffer[:end + 2]
                    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                    began = time.perf_counter()
                    result = self.detector.detect(frame)
                    processing_ms = (time.perf_counter() - began) * 1000.0
                    now = time.monotonic()
                    source_fps = 1.0 / max(.001, now - last_at)
                    last_at = now
                    if result.bbox is not None:
                        x, y, width, height = result.bbox
                        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 220, 0), 2)
                        cv2.putText(frame, f"FACE {width}px", (x, max(22, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 0), 2)
                    cv2.putText(frame, f"Haar face probe | detected={result.detected} | {processing_ms:.1f} ms", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 220, 255), 2)
                    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 78])
                    if not ok:
                        continue
                    frame_index += 1
                    self._record(frame_index, result, processing_ms, source_fps)
                    self._detections.append((now, result.detected))
                    while self._detections and self._detections[0][0] < now - 5.0:
                        self._detections.popleft()
                    detect_rate = 100.0 * sum(detected for _at, detected in self._detections) / max(1, len(self._detections))
                    with self._lock:
                        self._latest = encoded.tobytes()
                        self._status = {"detected": result.detected, "width": result.width, "area": result.area, "processing_ms": round(processing_ms, 1), "source_fps": round(source_fps, 1), "detect_rate_5s": round(detect_rate, 1), "frames": frame_index, "error": ""}
        except Exception as exc:
            with self._lock:
                self._status["error"] = str(exc)


def create_app(probe: FaceProbe) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return HTML

    @app.get("/status")
    def status():
        return jsonify(probe.snapshot()[1])

    @app.get("/video_feed")
    def video_feed():
        def generate():
            last = None
            while True:
                jpeg, _status = probe.snapshot()
                if jpeg is not None and jpeg != last:
                    last = jpeg
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                time.sleep(.03)
        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Observation-only Haar face probe")
    parser.add_argument("--source", default="http://127.0.0.1:5000/video_feed")
    parser.add_argument("--port", type=int, default=5058)
    parser.add_argument("--log", type=Path, default=Path(__file__).with_name("runtime_logs") / "face_probe.jsonl")
    args = parser.parse_args()
    probe = FaceProbe(args.source, args.log)
    probe.start()
    create_app(probe).run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
