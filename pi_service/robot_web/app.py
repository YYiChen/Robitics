from __future__ import annotations
import argparse
import atexit
import os
from pathlib import Path
from flask import Flask, Response, jsonify, request, render_template
from camera import CameraStreamer
from controller import RobotController
from dual_stream_camera import DualStreamCamera
from oled_status import OledStatusService
from system_metrics import SystemMetrics
from webrtc_stream import WebRTCStreamer

def create_app(controller: RobotController, camera: CameraStreamer | WebRTCStreamer | DualStreamCamera, system_metrics: SystemMetrics | None = None, oled: OledStatusService | None = None) -> Flask:
    app = Flask(__name__)
    system_metrics = system_metrics or SystemMetrics()
    def mjpeg_only():
        if camera.transport == "mjpeg": return None
        return jsonify(ok=False, error="当前为 H.264/WebRTC 模式；请使用 start_robot.sh 启动 MJPEG 后再调整该项"), 409
    def highres_available():
        if getattr(camera, "highres_available", False) or camera.transport == "mjpeg": return None
        return jsonify(ok=False, error="当前视频模式未启用高清 JPEG 通道"), 409
    @app.get("/")
    def index(): return render_template("index.html")
    @app.get("/video_feed")
    def video_feed():
        unavailable = mjpeg_only()
        if unavailable is not None: return unavailable
        if not camera.online: return jsonify(error=camera.error or camera.status), 503
        return Response(camera.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")
    @app.get("/highres_feed")
    def highres_feed():
        unavailable = highres_available()
        if unavailable is not None: return unavailable
        if not camera.online: return jsonify(error=camera.error or camera.status), 503
        return Response(camera.iter_highres_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")
    @app.get("/api/camera/highres/latest")
    def latest_highres_image():
        unavailable = highres_available()
        if unavailable is not None: return unavailable
        if not camera.online: return jsonify(error=camera.error or camera.status), 503
        jpeg = camera.latest_highres_jpeg()
        if jpeg is None: return jsonify(error="高清图片尚未生成"), 503
        return Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store, max-age=0"})
    @app.post("/api/action")
    def action():
        try: return jsonify(ok=True, action=controller.select_action((request.get_json(silent=True) or {}).get("action", "STOP")))
        except ValueError as exc: return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/keys")
    def keys():
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(
                ok=True,
                action=controller.update_keys(payload),
                deal=controller.deal_from_key_request(payload.get("deal_request")),
            )
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc:
            return jsonify(ok=False, error=str(exc)), 503
    @app.post("/api/heartbeat")
    def heartbeat(): controller.heartbeat(); return jsonify(ok=True)
    @app.post("/api/stop")
    def stop(): controller.stop_now(); return jsonify(ok=True)
    @app.post("/api/deal")
    def deal():
        payload = request.get_json(silent=True) or {}
        try: return jsonify(ok=True, state=controller.deal_card(payload.get("pwm", 255), payload.get("duration_ms", 1000)))
        except ValueError as exc: return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc: return jsonify(ok=False, error=str(exc)), 503
    @app.post("/api/feed")
    def feed():
        payload = request.get_json(silent=True) or {}
        try: return jsonify(ok=True, state=controller.feed_cards(payload.get("pwm", 255), payload.get("duration_ms", 5000)))
        except ValueError as exc: return jsonify(ok=False, error=str(exc)), 400
        except RuntimeError as exc: return jsonify(ok=False, error=str(exc)), 503
    @app.post("/api/servo")
    def servo():
        payload = request.get_json(silent=True) or {}
        try: return jsonify(ok=True, requested_angle=controller.set_servo_angle(payload.get("angle"), fast=bool(payload.get("fast", False))))
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
    @app.post("/api/camera/highres-profile")
    def camera_highres_profile():
        unavailable = highres_available()
        if unavailable is not None: return unavailable
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_highres_profile(str(payload.get("profile", ""))))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/camera/highres-fps")
    def camera_highres_fps():
        unavailable = highres_available()
        if unavailable is not None: return unavailable
        if not hasattr(camera, "set_highres_fps"):
            return jsonify(ok=False, error="当前相机后端不支持动态调整高清图片帧率"), 409
        payload = request.get_json(silent=True) or {}
        try:
            return jsonify(ok=True, camera=camera.set_highres_fps(payload.get("fps")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/camera/color-correction")
    def camera_color_correction():
        unavailable = mjpeg_only()
        if unavailable is not None: return unavailable
        try:
            return jsonify(ok=True, camera=camera.set_color_correction(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.get("/api/status")
    def status():
        return jsonify(
            api_version="robot-console-2026-07-24-card-combined-v4",
            capabilities={"system_metrics": True, "highres_fps_control": hasattr(camera, "set_highres_fps"), "highres_fps_max": 30, "card_deal": True, "card_feed": True},
            robot=controller.status(),
            camera=camera.status_dict(),
            system=system_metrics.status_dict(),
            oled=oled.status_dict() if oled else {"online": False, "disabled": True, "error": "已通过启动参数关闭"},
        )
    return app

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--drive-config", default=os.environ.get("ROBOT_DRIVE_CONFIG"))
    parser.add_argument("--web-port", type=int, default=5000)
    parser.add_argument("--video-backend", choices=("mjpeg", "webrtc"), default="mjpeg")
    parser.add_argument("--webrtc-width", type=int, default=1280)
    parser.add_argument("--webrtc-height", type=int, default=720)
    parser.add_argument("--webrtc-fps", type=float, default=30.0)
    parser.add_argument("--webrtc-bitrate", type=int, default=2_500_000)
    parser.add_argument("--webrtc-gop-frames", type=int, default=8)
    parser.add_argument("--webrtc-port", type=int, default=8889)
    parser.add_argument("--webrtc-path", default="cam")
    parser.add_argument("--highres-width", type=int, default=1640)
    parser.add_argument("--highres-height", type=int, default=1232)
    parser.add_argument("--webrtc-udp-output", default="udp://127.0.0.1:1234?pkt_size=1316")
    parser.add_argument("--disable-oled", action="store_true")
    parser.add_argument("--oled-address", type=lambda value: int(value, 0), default=0x3C)
    parser.add_argument("--oled-i2c-port", type=int, default=1)
    args = parser.parse_args()
    camera = (
        CameraStreamer()
        if args.video_backend == "mjpeg"
        else DualStreamCamera(
            video_width=args.webrtc_width, video_height=args.webrtc_height,
            video_fps=args.webrtc_fps, video_bitrate=args.webrtc_bitrate,
            webrtc_gop_frames=args.webrtc_gop_frames,
            highres_width=args.highres_width, highres_height=args.highres_height,
            webrtc_port=args.webrtc_port, webrtc_path=args.webrtc_path,
            udp_output=args.webrtc_udp_output,
        )
    )
    config_path = Path(args.drive_config).expanduser() if args.drive_config else None
    camera.start(); controller = RobotController(args.port, config_path=config_path); controller.start()
    oled = None if args.disable_oled else OledStatusService(controller, camera, address=args.oled_address, i2c_port=args.oled_i2c_port)
    if oled: oled.start()
    # Persist the current wheel profiles on normal process exit as well as
    # when the browser clicks the save button.  This covers Ctrl+C and service
    # shutdown, while RobotController.stop() remains idempotent.
    atexit.register(controller.stop)
    atexit.register(camera.stop)
    if oled: atexit.register(oled.stop)
    try:
        create_app(controller,camera,oled=oled).run(host="0.0.0.0",port=args.web_port,threaded=True,use_reloader=False)
    finally:
        controller.stop()
        camera.stop()
        if oled: oled.stop()
if __name__ == "__main__": main()
