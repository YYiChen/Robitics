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
from autonomous_route import AutonomousRouteTracker, AutonomousRunGate, RoutePreviewPublisher, load_tuning_config
from scanline_i_route import ScanlineIShapeRouteTracker, load_scanline_tuning_config
from green_white_scanline_i_route import GreenWhiteScanlineIShapeRouteTracker
from four_endpoint_validation_route import FourEndpointValidationRouteTracker
from pc_vision_adaptor_route import PcVisionAdaptorRouteTracker
from end_line_turn_adaptor import EndLineTurnAdaptorRouteTracker

def create_app(controller: RobotController, camera: CameraStreamer | WebRTCStreamer | DualStreamCamera, system_metrics: SystemMetrics | None = None, oled: OledStatusService | None = None, route_preview: RoutePreviewPublisher | None = None, route_tracker: AutonomousRouteTracker | None = None) -> Flask:
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
    @app.get("/route_preview_feed")
    def route_preview_feed():
        if route_preview is None:
            return jsonify(error="路线预判未通过启动参数开启"), 404
        return Response(route_preview.iter_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")
    @app.get("/api/vision-adaptor/frame")
    def vision_adaptor_frame():
        if not isinstance(route_tracker, PcVisionAdaptorRouteTracker):
            return jsonify(ok=False, error="当前 route mode 不是 pc_vision_adaptor"), 409
        jpeg, sequence, captured_at_ms, fast_center, fast_confidence, fast_centers = route_tracker.frame_snapshot()
        if jpeg is None:
            return jsonify(ok=False, error="Pi 尚未发布 adaptor 帧"), 503
        fast_rows = ";".join(f"{y},{x:.1f},{width}" for y, x, width in fast_centers)
        return Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store, max-age=0", "X-Vision-Frame-Seq": str(sequence), "X-Vision-Captured-At-Ms": str(captured_at_ms), "X-Vision-Pi-Fast-Centre": "" if fast_center is None else f"{fast_center:.1f}", "X-Vision-Pi-Fast-Confidence": f"{fast_confidence:.2f}", "X-Vision-Pi-Fast-Centers": fast_rows})
    @app.post("/api/vision-adaptor/event")
    def vision_adaptor_event():
        if not isinstance(route_tracker, PcVisionAdaptorRouteTracker):
            return jsonify(ok=False, error="当前 route mode 不是 pc_vision_adaptor"), 409
        try:
            return jsonify(ok=True, **route_tracker.submit_remote_event(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/vision-adaptor/preview")
    def vision_adaptor_preview():
        if not isinstance(route_tracker, PcVisionAdaptorRouteTracker):
            return jsonify(ok=False, error="当前 route mode 不是 pc_vision_adaptor"), 409
        try:
            return jsonify(ok=True, **route_tracker.submit_remote_preview(request.get_data(cache=False), request.headers))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/autonomous/toggle")
    def autonomous_toggle():
        if route_tracker is None:
            return jsonify(ok=False, error="路线预判未通过启动参数开启"), 409
        return jsonify(ok=True, autonomous=route_tracker.toggle_drive())
    @app.post("/api/autonomous/tuning")
    def autonomous_tuning():
        if route_tracker is None:
            return jsonify(ok=False, error="路线预判未通过启动参数开启"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.update_tuning(request.get_json(silent=True) or {}))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
    @app.post("/api/autonomous/manual-turn")
    def autonomous_manual_turn():
        if not isinstance(route_tracker, EndLineTurnAdaptorRouteTracker):
            return jsonify(ok=False, error="当前路线模式不支持 Q/E/U/I 视觉转向"), 409
        try:
            return jsonify(ok=True, autonomous=route_tracker.request_manual_turn((request.get_json(silent=True) or {}).get("command", "")))
        except ValueError as exc:
            return jsonify(ok=False, error=str(exc)), 400
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
    @app.post("/api/drive")
    def drive():
        payload = request.get_json(silent=True) or {}
        try:
            right, left = controller.set_direct_drive(payload.get("right_pwm"), payload.get("left_pwm"))
            return jsonify(ok=True, right_pwm=right, left_pwm=left)
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
        try: return jsonify(ok=True, state=controller.feed_cards(payload.get("pwm", -255), payload.get("duration_ms", 5000)))
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
            autonomous=route_tracker.status_dict() if route_tracker else {"available": False, "enabled": False, "state": "disabled", "detail": "未通过启动参数开启"},
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
    parser.add_argument("--enable-autonomous-route", action="store_true", help="run route preview inside the port-5000 service")
    parser.add_argument("--route-mode", choices=("generic", "scanline_i", "scanline_i_green_white", "scanline_i_four_endpoint_green_white", "pc_vision_adaptor", "end_line_turn_adaptor"), default="end_line_turn_adaptor", help="single-white-line/red-terminal adaptor, legacy PC adaptor, generic route, or isolated I-shape validations")
    parser.add_argument("--route-config", type=Path, default=Path(__file__).resolve().parents[2] / "third_party" / "DeskMate-Advance" / "src" / "track_line" / "config.fixed_green_white_course.json")
    parser.add_argument("--route-process-fps", type=float, default=20.0)
    parser.add_argument("--route-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "continuous_path_validation" / "tuning.py")
    parser.add_argument("--scanline-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "i_shape_scanline_turnaround_validation" / "scanline_web_tuning.json")
    parser.add_argument("--green-white-scanline-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "i_shape_green_white_turnaround_validation" / "scanline_web_tuning.json")
    parser.add_argument("--four-endpoint-scanline-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "i_shape_four_endpoint_navigation_validation" / "scanline_web_tuning.json")
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
    route_preview = RoutePreviewPublisher() if args.enable_autonomous_route else None
    route_gate = AutonomousRunGate() if args.enable_autonomous_route else None
    route_tracker = None
    if route_preview is not None and route_gate is not None:
        if args.route_mode == "end_line_turn_adaptor":
            route_tracker = EndLineTurnAdaptorRouteTracker(controller, camera, route_preview, route_gate)
        elif args.route_mode == "pc_vision_adaptor":
            # Default: Pi owns low-latency M1/M2 control; desktop reports only
            # authenticated high-level visual events through the adaptor API.
            route_tracker = PcVisionAdaptorRouteTracker(controller, camera, route_preview, route_gate)
        elif args.route_mode == "scanline_i":
            route_tracker = ScanlineIShapeRouteTracker(controller, camera, route_preview, route_gate, load_scanline_tuning_config(args.scanline_tuning))
        elif args.route_mode == "scanline_i_green_white":
            route_tracker = GreenWhiteScanlineIShapeRouteTracker(controller, camera, route_preview, route_gate, load_scanline_tuning_config(args.green_white_scanline_tuning))
        elif args.route_mode == "scanline_i_four_endpoint_green_white":
            route_tracker = FourEndpointValidationRouteTracker(controller, camera, route_preview, route_gate, load_scanline_tuning_config(args.four_endpoint_scanline_tuning))
        else:
            route_config = load_tuning_config(args.route_config, args.route_tuning)
            if args.route_process_fps != 20.0:
                route_config = route_config.__class__(**{**route_config.__dict__, "process_fps": args.route_process_fps})
            route_tracker = AutonomousRouteTracker(controller, camera, route_preview, route_gate, route_config)
        route_tracker.start()
    # Persist the current wheel profiles on normal process exit as well as
    # when the browser clicks the save button.  This covers Ctrl+C and service
    # shutdown, while RobotController.stop() remains idempotent.
    atexit.register(controller.stop)
    atexit.register(camera.stop)
    if oled: atexit.register(oled.stop)
    if route_tracker: atexit.register(route_tracker.stop)
    try:
        create_app(controller,camera,oled=oled,route_preview=route_preview,route_tracker=route_tracker).run(host="0.0.0.0",port=args.web_port,threaded=True,use_reloader=False)
    finally:
        controller.stop()
        camera.stop()
        if oled: oled.stop()
        if route_tracker: route_tracker.stop()
if __name__ == "__main__": main()
