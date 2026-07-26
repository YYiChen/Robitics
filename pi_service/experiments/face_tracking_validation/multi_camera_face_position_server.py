"""Computer-side concurrent face analysis for a Pi camera and an optional phone.

Each camera keeps its own pixel coordinate system.  The fused result chooses a
primary observation; it deliberately does not average incompatible offsets.
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
from face_position_server import face_payload

HTML = """<!doctype html><meta charset=utf-8><title>双相机人脸分析</title>
<style>body{margin:0;background:#09111d;color:#dbeafe;font:16px system-ui}main{max-width:1320px;margin:18px auto;padding:0 14px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px}.card{background:#132238;padding:12px;border-radius:10px}img{display:block;width:100%;background:#000}.note{color:#a9b9cb;white-space:pre-wrap}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style>
<main><h2>Pi + 手机：电脑端双相机人脸分析（无电机）</h2><div class=grid><section class=card><h3>树莓派相机</h3><img src=/video/pi></section><section class=card><h3>手机相机</h3><img src=/video/phone></section></div><pre id=status class=note>加载状态中…</pre></main>
<script>setInterval(async()=>{let r=await fetch('/api/faces/latest');document.querySelector('#status').textContent=JSON.stringify(await r.json(),null,2)},500)</script>"""


def choose_primary(sources: dict[str, dict]) -> dict:
    """Choose the closest/highest-confidence live view without mixing coordinates."""
    visible = [(name, item) for name, item in sources.items() if item.get("detected")]
    if not visible:
        return {"detected": False, "primary_source": None, "both_detected": False,
                "reason": "no_face_in_either_camera", "offset_x": None, "offset_y": None}
    name, item = max(visible, key=lambda pair: (pair[1]["box_width"] * pair[1]["box_height"], pair[1]["score"]))
    return {
        "detected": True,
        "primary_source": name,
        "both_detected": len(visible) == 2,
        "reason": "largest_face_box_then_confidence",
        "offset_x": item["offset_x"],
        "offset_y": item["offset_y"],
        "offset_x_normalized": item["offset_x_normalized"],
        "offset_y_normalized": item["offset_y_normalized"],
        "score": item["score"],
    }


class MultiCameraPublisher:
    def __init__(self, sources: dict[str, str]) -> None:
        self.sources = sources
        self._lock = threading.Lock()
        self._latest = {name: {"detected": False, "source": url, "error": "starting"} for name, url in sources.items()}
        self._latest_jpeg: dict[str, bytes | None] = {name: None for name in sources}

    def start(self) -> None:
        for name, url in self.sources.items():
            threading.Thread(target=self._run_source, args=(name, url), daemon=True,
                             name=f"face-camera-{name}").start()

    def snapshot(self) -> dict:
        with self._lock:
            sources = {name: dict(item) for name, item in self._latest.items()}
        return {
            "time": datetime.now(timezone.utc).isoformat(),
            "sources": sources,
            "fused": choose_primary(sources),
            "note": "Offsets are camera-local; fusion selects one primary source and does not average them.",
        }

    def frame(self, name: str) -> bytes | None:
        with self._lock:
            return self._latest_jpeg.get(name)

    def _update(self, name: str, payload: dict) -> None:
        with self._lock:
            self._latest[name] = payload

    def _run_source(self, name: str, url: str) -> None:
        detector = FaceDetector()
        capture, frame_index, previous_at = None, 0, time.monotonic()
        try:
            while True:
                if capture is None or not capture.isOpened():
                    capture = cv2.VideoCapture(url)
                    if not capture.isOpened():
                        self._update(name, {"time": datetime.now(timezone.utc).isoformat(), "detected": False, "source": url, "error": "cannot_open_video_source"})
                        time.sleep(1.0)
                        continue
                ok, frame = capture.read()
                if not ok:
                    capture.release()
                    capture = None
                    self._update(name, {"time": datetime.now(timezone.utc).isoformat(), "detected": False, "source": url, "error": "video_source_read_failed"})
                    time.sleep(.25)
                    continue
                began = time.perf_counter()
                result = detector.detect(frame)
                processing_ms = (time.perf_counter() - began) * 1000.0
                now = time.monotonic()
                source_fps = 1.0 / max(.001, now - previous_at)
                previous_at = now
                frame_index += 1
                payload = face_payload(result, frame=frame_index, processing_ms=processing_ms,
                                       source_fps=source_fps, source=url)
                if result.detected:
                    x = round(result.center_x - result.box_width / 2)
                    y = round(result.center_y - result.box_height / 2)
                    cv2.rectangle(frame, (x, y), (x + result.box_width, y + result.box_height), (0, 220, 0), 2)
                    cv2.putText(frame, f"FACE {result.score:.0%}  offX={result.offset_x:.0f}", (x, max(22, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 220, 0), 2)
                else:
                    cv2.putText(frame, "NO FACE", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 180, 255), 2)
                ok_jpeg, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self._update(name, payload)
                if ok_jpeg:
                    with self._lock:
                        self._latest_jpeg[name] = encoded.tobytes()
        except Exception as exc:
            self._update(name, {"time": datetime.now(timezone.utc).isoformat(), "detected": False, "source": url, "error": f"publisher_error:{exc}"})
        finally:
            if capture is not None:
                capture.release()
            detector.close()


def make_handler(publisher: MultiCameraPublisher):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            if self.path == "/":
                body = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith("/video/"):
                name = self.path.removeprefix("/video/")
                if name not in publisher.sources:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = None
                try:
                    while True:
                        jpeg = publisher.frame(name)
                        if jpeg is not None and jpeg != last:
                            last = jpeg
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                        time.sleep(.03)
                except (BrokenPipeError, ConnectionResetError):
                    return
                return
            if self.path not in {"/health", "/api/faces/latest"}:
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
    parser = argparse.ArgumentParser(description="Computer-side concurrent Pi + phone face analysis")
    parser.add_argument("--pi-source", default="http://100.80.46.54:5000/video_feed")
    parser.add_argument("--phone-source", required=True, help="for example: http://10.50.77.86:8080/video")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5060)
    args = parser.parse_args()
    publisher = MultiCameraPublisher({"pi": args.pi_source, "phone": args.phone_source})
    publisher.start()
    print(f"Concurrent face JSON: http://{args.host}:{args.port}/api/faces/latest")
    print("Computer inference only; no robot, motor, Arduino, or route-control imports.")
    ThreadingHTTPServer((args.host, args.port), make_handler(publisher)).serve_forever()


if __name__ == "__main__":
    main()
