from __future__ import annotations
import argparse
import atexit
from flask import Flask, Response, jsonify, request, render_template
from camera import CameraStreamer
from controller import RobotController
from system_metrics import SystemMetrics
from webrtc_stream import WebRTCStreamer

def create_app(controller: RobotController, camera: CameraStreamer | WebRTCStreamer, system_metrics: SystemMetrics | None = None) -> Flask:
    app = Flask(__name__)
    system_metrics = system_metrics or SystemMetrics()
    def mjpeg_only():
        if camera.transport == "mjpeg": return None
        return jsonify(ok=False, error="当前为 H.264/WebRTC 模式；请使用 start_robot.sh 启动 MJPEG 后再调整该项"), 409
    @app.get("/")
    def index(): return render_template("index.html")
    @app.get("/video_feed")
    def video_feed():
        unavailable = mjpeg_only()
        if unavailable is not None: return unavailable
        if not camera.online: return jsonify(error=camera.error or camera.status), 503
        return Response(camera.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")
    @app.post("/api/action")
    def action():
        try: return jsonify(ok=True, action=controller.select_action((request.get_json(silent=True) or {}).get("action", "STOP")))
        except ValueError as exc: return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/keys")
    def keys():
        return jsonify(ok=True, action=controller.update_keys(request.get_json(silent=True) or {}))
    @app.post("/api/heartbeat")
    def heartbeat(): controller.heartbeat(); return jsonify(ok=True)
    @app.post("/api/stop")
    def stop(): controller.stop_now(); return jsonify(ok=True)
    @app.post("/api/servo")
    def servo():
        payload = request.get_json(silent=True) or {}
        try: return jsonify(ok=True, requested_angle=controller.set_servo_angle(payload.get("angle")))
        except ValueError as exc: return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc: return jsonify(ok=False, error=str(exc)), 503
    @app.post("/api/reconnect")
    def reconnect():
        status = controller.reconnect()
        return jsonify(ok=status["arduino_online"], robot=status)
    @app.post("/api/config")
    def config(): return jsonify(ok=True, config=controller.update_config(request.get_json(silent=True) or {}))
    @app.post("/api/camera/mode")
    def camera_mode():
        unavailable = mjpeg_only()
        if unavailable is not None: return unavailable
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_mode(str(payload.get("mode", ""))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc), camera=camera.status_dict()), 503
    @app.post("/api/camera/exposure")
    def camera_exposure():
        unavailable = mjpeg_only()
        if unavailable is not None: return unavailable
        try:
            return jsonify(ok=True, camera=camera.set_exposure(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc), camera=camera.status_dict()), 503
    @app.post("/api/camera/stream-profile")
    def camera_stream_profile():
        unavailable = mjpeg_only()
        if unavailable is not None: return unavailable
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_stream_profile(str(payload.get("profile", ""))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.get("/api/status")
    def status(): return jsonify(robot=controller.status(), camera=camera.status_dict(), system=system_metrics.status_dict())
    return app

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--web-port", type=int, default=5000)
    parser.add_argument("--video-backend", choices=("mjpeg", "webrtc"), default="mjpeg")
    parser.add_argument("--webrtc-width", type=int, default=1280)
    parser.add_argument("--webrtc-height", type=int, default=720)
    parser.add_argument("--webrtc-fps", type=float, default=30.0)
    parser.add_argument("--webrtc-bitrate", type=int, default=2_500_000)
    parser.add_argument("--webrtc-port", type=int, default=8889)
    parser.add_argument("--webrtc-path", default="cam")
    args = parser.parse_args()
    camera = (
        CameraStreamer()
        if args.video_backend == "mjpeg"
        else WebRTCStreamer(args.webrtc_width, args.webrtc_height, args.webrtc_fps, args.webrtc_bitrate, args.webrtc_port, args.webrtc_path)
    )
    camera.start(); controller = RobotController(args.port); controller.start()
    # Persist the current wheel profiles on normal process exit as well as
    # when the browser clicks the save button.  This covers Ctrl+C and service
    # shutdown, while RobotController.stop() remains idempotent.
    atexit.register(controller.stop)
    atexit.register(camera.stop)
    try:
        create_app(controller,camera).run(host="0.0.0.0",port=args.web_port,threaded=True,use_reloader=False)
    finally:
        controller.stop()
        camera.stop()
if __name__ == "__main__": main()
