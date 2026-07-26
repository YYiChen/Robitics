"""Flask service assembly and command-line entry point."""
from __future__ import annotations

import argparse
import atexit
import os
from pathlib import Path

from flask import Flask

from api import (
    register_camera_api,
    register_control_api,
    register_route_api,
    register_status_api,
)
from autonomous_route import (
    AutonomousRouteTracker,
    AutonomousRunGate,
    RoutePreviewPublisher,
    load_tuning_config,
)
from camera import CameraStreamer
from configuration import UnifiedConfigStore, migrate_formal_legacy_config
from controller import RobotController
from dual_stream_camera import DualStreamCamera
from four_endpoint_validation_route import FourEndpointValidationRouteTracker
from green_white_scanline_i_route import GreenWhiteScanlineIShapeRouteTracker
from oled_status import OledStatusService
from pc_vision_adaptor_route import PcVisionAdaptorRouteTracker
from routes.end_line.tracker import EndLineTurnAdaptorRouteTracker
from scanline_i_route import ScanlineIShapeRouteTracker, load_scanline_tuning_config
from system_metrics import SystemMetrics
from webrtc_stream import WebRTCStreamer


def create_app(
    controller: RobotController,
    camera: CameraStreamer | WebRTCStreamer | DualStreamCamera,
    system_metrics: SystemMetrics | None = None,
    oled: OledStatusService | None = None,
    route_preview: RoutePreviewPublisher | None = None,
    route_tracker: AutonomousRouteTracker | None = None,
) -> Flask:
    """Assemble endpoint groups without owning runtime lifecycle."""
    app = Flask(__name__)
    metrics = system_metrics or SystemMetrics()
    register_camera_api(app, camera)
    register_control_api(app, controller)
    register_route_api(app, route_preview, route_tracker)
    register_status_api(app, controller, camera, metrics, oled, route_tracker)
    return app


def build_argument_parser() -> argparse.ArgumentParser:
    """Define CLI paths and modes separately from service startup."""
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
    parser.add_argument(
        "--route-mode",
        choices=("generic", "scanline_i", "scanline_i_green_white", "scanline_i_four_endpoint_green_white", "pc_vision_adaptor", "end_line_turn_adaptor"),
        default="end_line_turn_adaptor",
        help="single-white-line/red-terminal adaptor, legacy PC adaptor, generic route, or isolated I-shape validations",
    )
    parser.add_argument("--route-config", type=Path, default=Path(__file__).resolve().parents[2] / "third_party" / "DeskMate-Advance" / "src" / "track_line" / "config.fixed_green_white_course.json")
    parser.add_argument("--route-process-fps", type=float, default=20.0)
    parser.add_argument("--route-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "continuous_path_validation" / "tuning.py")
    parser.add_argument("--scanline-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "i_shape_scanline_turnaround_validation" / "scanline_web_tuning.json")
    parser.add_argument("--green-white-scanline-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "i_shape_green_white_turnaround_validation" / "scanline_web_tuning.json")
    parser.add_argument("--four-endpoint-scanline-tuning", type=Path, default=Path(__file__).resolve().parents[1] / "experiments" / "i_shape_four_endpoint_navigation_validation" / "scanline_web_tuning.json")
    parser.add_argument("--oled-address", type=lambda value: int(value, 0), default=0x3C)
    parser.add_argument("--oled-i2c-port", type=int, default=1)
    return parser


def build_camera(args, config_store: UnifiedConfigStore):
    if args.video_backend == "mjpeg":
        return CameraStreamer(config_store=config_store)
    return DualStreamCamera(
        video_width=args.webrtc_width,
        video_height=args.webrtc_height,
        video_fps=args.webrtc_fps,
        video_bitrate=args.webrtc_bitrate,
        webrtc_gop_frames=args.webrtc_gop_frames,
        highres_width=args.highres_width,
        highres_height=args.highres_height,
        webrtc_port=args.webrtc_port,
        webrtc_path=args.webrtc_path,
        udp_output=args.webrtc_udp_output,
        config_store=config_store,
    )


def build_route_tracker(args, controller, camera, publisher, gate, config_store):
    if publisher is None or gate is None:
        return None
    if args.route_mode == "end_line_turn_adaptor":
        return EndLineTurnAdaptorRouteTracker(
            controller, camera, publisher, gate, config_store=config_store
        )
    if args.route_mode == "pc_vision_adaptor":
        return PcVisionAdaptorRouteTracker(controller, camera, publisher, gate)
    if args.route_mode == "scanline_i":
        return ScanlineIShapeRouteTracker(
            controller, camera, publisher, gate,
            load_scanline_tuning_config(args.scanline_tuning),
        )
    if args.route_mode == "scanline_i_green_white":
        return GreenWhiteScanlineIShapeRouteTracker(
            controller, camera, publisher, gate,
            load_scanline_tuning_config(args.green_white_scanline_tuning),
        )
    if args.route_mode == "scanline_i_four_endpoint_green_white":
        return FourEndpointValidationRouteTracker(
            controller, camera, publisher, gate,
            load_scanline_tuning_config(args.four_endpoint_scanline_tuning),
        )
    route_config = load_tuning_config(args.route_config, args.route_tuning)
    if args.route_process_fps != 20.0:
        route_config = route_config.__class__(
            **{**route_config.__dict__, "process_fps": args.route_process_fps}
        )
    return AutonomousRouteTracker(controller, camera, publisher, gate, route_config)


def main() -> None:
    args = build_argument_parser().parse_args()
    config_store = UnifiedConfigStore()
    migrate_formal_legacy_config(config_store)
    camera = build_camera(args, config_store)
    config_path = Path(args.drive_config).expanduser() if args.drive_config else None
    controller = RobotController(
        args.port, config_path=config_path, config_store=config_store
    )
    camera.start()
    controller.start()
    oled = None if args.disable_oled else OledStatusService(
        controller, camera, address=args.oled_address, i2c_port=args.oled_i2c_port
    )
    if oled:
        oled.start()
    route_preview = RoutePreviewPublisher() if args.enable_autonomous_route else None
    route_gate = AutonomousRunGate() if args.enable_autonomous_route else None
    route_tracker = build_route_tracker(
        args, controller, camera, route_preview, route_gate, config_store
    )
    if route_tracker:
        route_tracker.start()

    atexit.register(controller.stop)
    atexit.register(camera.stop)
    if oled:
        atexit.register(oled.stop)
    if route_tracker:
        atexit.register(route_tracker.stop)
    try:
        create_app(
            controller, camera, oled=oled,
            route_preview=route_preview, route_tracker=route_tracker,
        ).run(host="0.0.0.0", port=args.web_port, threaded=True, use_reloader=False)
    finally:
        controller.stop()
        camera.stop()
        if oled:
            oled.stop()
        if route_tracker:
            route_tracker.stop()


if __name__ == "__main__":
    main()
