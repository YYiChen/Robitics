"""Minimal whole-car test page: CSI MJPEG preview plus M1/M2 drive heartbeat."""
from __future__ import annotations

import argparse
import glob
import sys
import threading
import time
from pathlib import Path

import serial
from flask import Flask, Response, jsonify, render_template, request

# Reuse the tested CSI capture implementation from the formal robot service.
ROBOT_WEB_DIR = Path(__file__).resolve().parents[1] / "robot_web"
sys.path.insert(0, str(ROBOT_WEB_DIR))
from camera import CameraStreamer  # noqa: E402

HEARTBEAT_SECONDS = 0.15
CLIENT_TIMEOUT_SECONDS = 0.55
VALID_ACTIONS = {"F", "B", "L", "R", "STOP"}


class DriveBridge:
    def __init__(self, port: str, baud: int = 9600) -> None:
        self.port_name = port
        self.baud = baud
        self.port: serial.Serial | None = None
        self.lock = threading.RLock()
        self.action = "STOP"
        self.last_client_seen = 0.0
        self.last_reply = ""
        self.error = ""
        self.running = threading.Event()
        self.thread: threading.Thread | None = None

    def _connect(self) -> None:
        with self.lock:
            if self.port and self.port.is_open:
                return
            candidates = [] if self.port_name == "auto" else [self.port_name]
            candidates += sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
            candidates = list(dict.fromkeys(candidates))
            failures = []
            for candidate in candidates:
                try:
                    self.port = serial.Serial(candidate, self.baud, timeout=0.02, write_timeout=0.5)
                    self.port_name = candidate
                    time.sleep(2.2)  # USB serial opening commonly resets an Arduino.
                    self.error = ""
                    return
                except Exception as exc:
                    failures.append(f"{candidate}: {exc}")
                    self.port = None
            self.error = "; ".join(failures) if failures else "no /dev/ttyACM* or /dev/ttyUSB* device found"

    def _write(self, command: str) -> None:
        self._connect()
        with self.lock:
            if not self.port:
                return
            try:
                self.port.write((command + "\n").encode("ascii"))
                self.port.flush()
            except Exception as exc:
                self.error = str(exc)
                self.port = None

    def select_action(self, action: str) -> str:
        action = action.upper()
        if action not in VALID_ACTIONS:
            raise ValueError("action must be F, B, L, R, or STOP")
        with self.lock:
            self.action = action
            self.last_client_seen = time.monotonic()
        if action == "STOP":
            self._write("STOP")
        return action

    def set_speed(self, raw: object) -> int:
        speed = max(0, min(255, int(raw)))
        self._write(f"SPD,{speed}")
        return speed

    def _loop(self) -> None:
        while self.running.is_set():
            with self.lock:
                active = self.action if time.monotonic() - self.last_client_seen <= CLIENT_TIMEOUT_SECONDS else "STOP"
                if active == "STOP":
                    self.action = "STOP"
            self._write(active)
            with self.lock:
                if self.port:
                    try:
                        line = self.port.readline().decode("ascii", "ignore").strip()
                        if line:
                            self.last_reply = line
                    except Exception as exc:
                        self.error = str(exc)
            time.sleep(HEARTBEAT_SECONDS)

    def start(self) -> None:
        self.running.set()
        self.thread = threading.Thread(target=self._loop, name="drive-heartbeat", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running.clear()
        self._write("STOP")
        if self.thread:
            self.thread.join(timeout=1.0)
        with self.lock:
            if self.port:
                self.port.close()
                self.port = None

    def status(self) -> dict[str, object]:
        with self.lock:
            return {"serial": bool(self.port and self.port.is_open), "action": self.action,
                    "reply": self.last_reply, "error": self.error, "port": self.port_name}


def create_app(bridge: DriveBridge, camera: CameraStreamer) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/video_feed")
    def video_feed():
        if not camera.online:
            return jsonify(error=camera.error or "camera offline"), 503
        return Response(camera.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.post("/api/action")
    def action():
        try:
            return jsonify(ok=True, action=bridge.select_action((request.get_json(silent=True) or {}).get("action", "STOP")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400

    @app.post("/api/speed")
    def speed():
        try:
            return jsonify(ok=True, speed=bridge.set_speed((request.get_json(silent=True) or {}).get("speed", 100)))
        except (TypeError, ValueError):
            return jsonify(ok=False, error="speed must be 0..255"), 400

    @app.get("/api/status")
    def status():
        return jsonify(robot=bridge.status(), camera=camera.status_dict())

    return app


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="auto", help="serial device path, or auto")
    parser.add_argument("--web-port", type=int, default=5050)
    args = parser.parse_args()
    camera = CameraStreamer()
    bridge = DriveBridge(args.port)
    camera.start()
    bridge.start()
    app = create_app(bridge, camera)
    try:
        app.run(host="0.0.0.0", port=args.web_port, threaded=True)
    finally:
        bridge.stop()
        camera.stop()


if __name__ == "__main__":
    main()
