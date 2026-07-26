"""Computer-side MediaPipe service that publishes face position as JSON only.

It reads the Raspberry Pi MJPEG stream, performs all inference on this computer,
and never sends a request to the robot or Arduino.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time

import cv2

from face_detector import FaceDetectionResult, FaceDetector


def capture_source(source: str) -> str | int:
    """Treat a decimal CLI source such as ``0`` as a local Windows camera."""
    return int(source) if source.strip().isdigit() else source


def face_payload(result: FaceDetectionResult, *, frame: int, processing_ms: float,
                 source_fps: float, source: str, error: str = "") -> dict:
    """Stable wire format for a future Pi-side consumer."""
    return {
        "time": datetime.now(timezone.utc).isoformat(),
        "frame": frame,
        "source": source,
        "detected": result.detected,
        "center_x": result.center_x,
        "center_y": result.center_y,
        "offset_x": result.offset_x,
        "offset_y": result.offset_y,
        "offset_x_normalized": None if result.offset_x is None else round(result.offset_x / (result.frame_width / 2), 4),
        "offset_y_normalized": None if result.offset_y is None else round(result.offset_y / (result.frame_height / 2), 4),
        "box_width": result.box_width,
        "box_height": result.box_height,
        "score": round(result.score, 4),
        "frame_width": result.frame_width,
        "frame_height": result.frame_height,
        "processing_ms": round(processing_ms, 2),
        "source_fps": round(source_fps, 2),
        "error": error,
    }


class FacePositionPublisher:
    def __init__(self, source: str) -> None:
        self.source = source
        self._capture_source = capture_source(source)
        self._lock = threading.Lock()
        self._latest = {"detected": False, "source": source, "error": "starting"}

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def start(self) -> None:
        threading.Thread(target=self._run, daemon=True, name="face-position-publisher").start()

    def _set_error(self, error: str) -> None:
        with self._lock:
            self._latest = {"time": datetime.now(timezone.utc).isoformat(), "detected": False, "source": self.source, "error": error}

    def _run(self) -> None:
        detector = FaceDetector()
        capture = None
        frame_index, previous_at = 0, time.monotonic()
        try:
            while True:
                if capture is None or not capture.isOpened():
                    capture = cv2.VideoCapture(self._capture_source)
                    if not capture.isOpened():
                        self._set_error("cannot_open_video_source")
                        time.sleep(1.0)
                        continue
                ok, frame = capture.read()
                if not ok:
                    capture.release()
                    capture = None
                    self._set_error("video_source_read_failed")
                    time.sleep(0.25)
                    continue
                began = time.perf_counter()
                result = detector.detect(frame)
                processing_ms = (time.perf_counter() - began) * 1000.0
                now = time.monotonic()
                source_fps = 1.0 / max(0.001, now - previous_at)
                previous_at = now
                frame_index += 1
                payload = face_payload(result, frame=frame_index, processing_ms=processing_ms,
                                       source_fps=source_fps, source=self.source)
                with self._lock:
                    self._latest = payload
        except Exception as exc:
            self._set_error(f"publisher_error:{exc}")
        finally:
            if capture is not None:
                capture.release()
            detector.close()


def make_handler(publisher: FacePositionPublisher):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            if self.path not in {"/health", "/api/face/latest"}:
                self.send_error(404)
                return
            body = json.dumps(publisher.snapshot(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):  # noqa: A003
            return
    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description="Computer-side MediaPipe face-position JSON publisher")
    parser.add_argument("--source", default="http://100.80.46.54:5000/video_feed", help="MJPEG URL, or a local camera index such as 0 for DroidCam")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5059)
    args = parser.parse_args()
    publisher = FacePositionPublisher(args.source)
    publisher.start()
    print(f"Face JSON publisher: http://{args.host}:{args.port}/api/face/latest")
    print("Observation only: no robot, motor, Arduino, or route-control imports.")
    ThreadingHTTPServer((args.host, args.port), make_handler(publisher)).serve_forever()


if __name__ == "__main__":
    main()
