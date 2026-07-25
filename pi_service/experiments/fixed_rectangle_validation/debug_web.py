"""Browser preview owned by the fixed-rectangle experiment."""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from urllib.parse import urlparse

import cv2


class DebugMjpegPublisher:
    def __init__(self, port: int) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._status: dict[str, object] = {"state": "waiting_for_first_frame"}
        self._running = True
        publisher = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                return

            def do_GET(self):  # noqa: N802
                path = urlparse(self.path).path
                if path == "/":
                    body = (
                        "<!doctype html><meta charset='utf-8'><title>Fixed Rectangle</title>"
                        "<style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif}"
                        "main{max-width:960px;margin:auto;padding:12px}img{width:100%;display:block}p{color:#9ecbff}</style>"
                        "<main><h2>固定顺时针矩形</h2><p>丢线后固定前进、固定右转、重新找线。</p>"
                        "<p id='status'>等待第一帧…</p><img src='/video_feed' alt='fixed rectangle debug stream'></main>"
                        "<script>setInterval(async()=>{try{const s=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json());"
                        "document.getElementById('status').textContent='实时预览帧 '+(s.preview_sequence??0)+' | 处理时间 '+(s.wall_time??'等待中')}catch(e){}},500)</script>"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/api/status":
                    with publisher._condition:
                        status = {**publisher._status, "preview_sequence": publisher._sequence}
                        body = json.dumps(status, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path != "/video_feed":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                sequence = 0
                try:
                    while True:
                        with publisher._condition:
                            publisher._condition.wait_for(lambda: not publisher._running or publisher._sequence != sequence, timeout=1.0)
                            if not publisher._running:
                                return
                            jpeg, sequence = publisher._jpeg, publisher._sequence
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        # Make each processed frame visible immediately instead of waiting
                        # for an HTTP buffer to fill on a slow browser connection.
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return

        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def publish(self, frame, status: dict[str, object]) -> None:
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._condition:
            self._jpeg = encoded.tobytes()
            self._status = dict(status)
            self._sequence += 1
            self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            self._running = False
            self._condition.notify_all()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=1.0)
