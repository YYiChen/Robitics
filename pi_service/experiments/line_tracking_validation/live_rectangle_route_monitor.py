"""Live MJPEG monitor: OpenCV line detection plus clockwise-route intent.

The program is display-only. It never opens a serial port and never sends a
motor command. Press Q or Esc in the preview window to stop it.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[2]
PI_SERVICE_ROOT = REPOSITORY_ROOT / "pi_service"
TRACK_LINE_SRC = REPOSITORY_ROOT / "third_party" / "DeskMate-Advance" / "src"
if not TRACK_LINE_SRC.is_dir():
    raise RuntimeError(f"track_line source is missing: {TRACK_LINE_SRC}")
sys.path.insert(0, str(TRACK_LINE_SRC))
sys.path.insert(0, str(REPOSITORY_ROOT))

from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402

from rectangle_route_planner import (  # noqa: E402
    ClockwiseRectanglePlanner,
    PlannerDecision,
    RectanglePlannerConfig,
)
from motor_control import MotorControlConfig, RobotWebMotorExecutor  # noqa: E402
from ground_plane import GroundPlaneLineFilter  # noqa: E402


DEFAULT_SOURCE = "http://100.80.46.54:5000/video_feed"
DEFAULT_CONFIG = TRACK_LINE_SRC / "track_line" / "config.dark_line.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=DEFAULT_SOURCE, help="MJPEG URL, video path, or camera index")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="OpenCV detector JSON config")
    parser.add_argument(
        "--process-fps",
        type=float,
        default=30.0,
        help="maximum frames per second to analyse; default 30, zero means analyse every frame",
    )
    parser.add_argument(
        "--track-roi-top-width",
        type=float,
        default=0.35,
        help="fractional width of the track trapezoid at the detector ROI top",
    )
    parser.add_argument(
        "--track-roi-bottom-width",
        type=float,
        default=0.56,
        help="fractional width of the track trapezoid at the bottom of the frame",
    )
    parser.add_argument("--headless", action="store_true", help="print JSON without preview windows")
    parser.add_argument(
        "--debug-web-port",
        type=int,
        default=0,
        help="serve the annotated monitoring view on this port; zero disables it",
    )
    parser.add_argument("--max-frames", type=int, default=0, help="stop after N analysed frames; zero means run until Q/Esc")
    parser.add_argument("--enable-motors", action="store_true", help="enable real motor commands; omitted means display-only")
    parser.add_argument("--controller-url", default="http://100.80.46.54:5000", help="Pi robot-web base URL")
    return parser.parse_args()


class DebugMjpegPublisher:
    """Serve the same annotated frame shown by the local OpenCV preview."""

    def __init__(self, port: int) -> None:
        self._condition = threading.Condition()
        self._jpeg: bytes | None = None
        self._sequence = 0
        self._status: dict[str, object] = {"state": "waiting_for_first_frame"}
        self._running = True
        publisher = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                # A browser requests a new MJPEG chunk for every rendered
                # frame; keep that normal traffic out of the motor terminal.
                return

            def do_GET(self):  # noqa: N802 - required by BaseHTTPRequestHandler
                path = urlparse(self.path).path
                if path == "/":
                    body = (
                        "<!doctype html><html><head><meta charset='utf-8'>"
                        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                        "<title>Pi Line Tracking</title>"
                        "<style>body{margin:0;background:#111;color:#ddd;font-family:sans-serif}"
                        "main{max-width:960px;margin:auto;padding:12px}img{width:100%;height:auto;display:block}"
                        "p{color:#9ecbff}</style></head><body><main>"
                        "<h2>树莓派循迹实时判断</h2><p>路线、偏差和电机动作由树莓派本机计算。</p>"
                        "<img src='/video_feed' alt='line tracking debug stream'></main></body></html>"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/api/status":
                    with publisher._condition:
                        body = json.dumps(publisher._status, ensure_ascii=False).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path != "/video_feed":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                sequence = 0
                try:
                    while True:
                        with publisher._condition:
                            publisher._condition.wait_for(
                                lambda: not publisher._running or publisher._sequence != sequence,
                                timeout=1.0,
                            )
                            if not publisher._running:
                                return
                            jpeg = publisher._jpeg
                            sequence = publisher._sequence
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return

        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="line-tracking-debug-web",
            daemon=True,
        )
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


def source_value(source: str) -> int | str:
    source = source.strip()
    return int(source) if source.lstrip("-").isdigit() else source


def planner_config_for_processing_rate(process_fps: float) -> RectanglePlannerConfig:
    """Keep physical corner timings stable when the analysis rate changes."""
    # With --process-fps 0 the source rate is not known in advance; retain the
    # original 10 FPS timing rather than silently making safety timing tiny.
    effective_fps = process_fps if process_fps > 0 else 10.0
    return RectanglePlannerConfig(
        missing_before_turn=max(1, round(0.05 * effective_fps)),
        recovery_forward_frames=max(1, round(0.3 * effective_fps)),
        max_turn_frames=max(1, round(10.0 * effective_fps)),
    )


def track_corridor(frame, *, roi_top_ratio: float, top_width: float, bottom_width: float):
    """Keep only the floor corridor in which the guide line may appear.

    The dark-line detector treats any black object as a candidate. Pixels
    outside this trapezoid are replaced with the observed floor's median
    colour, so chair wheels and table legs cannot become far-band line points
    without distorting Otsu's global brightness threshold.
    """
    if not 0 < top_width <= 1 or not 0 < bottom_width <= 1:
        raise ValueError("track ROI widths must be in (0, 1]")
    height, width = frame.shape[:2]
    top_y = int(round(height * roi_top_ratio))
    top_half = int(round(width * top_width / 2))
    bottom_half = int(round(width * bottom_width / 2))
    centre_x = width // 2
    polygon = np.asarray(
        [
            (centre_x - top_half, top_y),
            (centre_x + top_half, top_y),
            (centre_x + bottom_half, height - 1),
            (centre_x - bottom_half, height - 1),
        ],
        dtype=np.int32,
    )
    inside = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(inside, polygon, 255)
    floor_sample = frame[top_y:, width // 5 : width * 4 // 5]
    floor_colour = np.median(floor_sample.reshape(-1, 3), axis=0).astype(np.uint8)
    corridor = np.full_like(frame, floor_colour)
    corridor[inside > 0] = frame[inside > 0]
    return corridor, polygon


def draw_track_corridor(frame, polygon):
    output = frame.copy()
    cv2.polylines(output, [polygon], True, (255, 180, 0), 2, cv2.LINE_AA)
    return output


def keep_near_connected_points(result, frame_shape):
    """Reject band points not connected to the near guide-line component.

    The base detector selects candidates independently in three horizontal
    bands. A chair leg may therefore become the far point even when the near
    and middle points are correct. A real L corner is a single connected tape
    component, so all accepted points must share the near point's component.
    """
    if len(result.points_px) < 2:
        return result

    _count, labels = cv2.connectedComponents(result.mask, connectivity=8)
    labels_for_points: list[int] = []
    for x, y in result.points_px:
        mask_y = y - result.roi_top
        if not (0 <= mask_y < labels.shape[0] and 0 <= x < labels.shape[1]):
            labels_for_points.append(0)
            continue
        window = labels[
            max(0, mask_y - 8) : min(labels.shape[0], mask_y + 9),
            max(0, x - 8) : min(labels.shape[1], x + 9),
        ]
        foreground = window[window > 0]
        if foreground.size == 0:
            labels_for_points.append(0)
            continue
        candidates, counts = np.unique(foreground, return_counts=True)
        labels_for_points.append(int(candidates[np.argmax(counts)]))

    near_label = labels_for_points[-1]
    keep_indices = [
        index for index, label in enumerate(labels_for_points) if near_label and label == near_label
    ]
    if len(keep_indices) == len(result.points_px):
        return result

    kept_points = tuple(result.points_px[index] for index in keep_indices)
    kept_normalized = tuple(
        result.observation.points_normalized[index] for index in keep_indices
    )
    if len(kept_points) < 2:
        observation = replace(
            result.observation,
            offset=None,
            heading=None,
            curvature=None,
            confidence=0.0,
            line_lost=True,
            valid_bands=len(kept_points),
            points_normalized=kept_normalized,
            rejection_reason="disconnected_line_candidates",
        )
    else:
        frame_height, frame_width = frame_shape[:2]
        half_width = max(1.0, frame_width / 2.0)
        centres = [point[0] for point in kept_points]
        offset = float(np.clip((float(np.mean(centres)) - half_width) / half_width, -1, 1))
        heading = float(np.clip((centres[0] - centres[-1]) / half_width, -1, 1))
        observation = replace(
            result.observation,
            offset=offset,
            heading=heading,
            curvature=0.0 if len(kept_points) < 3 else result.observation.curvature,
            valid_bands=len(kept_points),
            points_normalized=kept_normalized,
            rejection_reason="far_candidate_disconnected",
        )
    return replace(result, observation=observation, points_px=kept_points)


def _component_label_near_point(labels, result) -> int:
    """Return the dominant foreground component near the closest line point."""
    if not result.points_px:
        return 0
    x, y = result.points_px[-1]
    mask_y = y - result.roi_top
    window = labels[
        max(0, mask_y - 8) : min(labels.shape[0], mask_y + 9),
        max(0, x - 8) : min(labels.shape[1], x + 9),
    ]
    foreground = window[window > 0]
    if foreground.size == 0:
        return 0
    candidates, counts = np.unique(foreground, return_counts=True)
    return int(candidates[np.argmax(counts)])


def has_connected_right_branch(result, *, minimum_width_ratio: float = 0.12) -> bool:
    """Recognise a horizontal right arm joined to the current near guide line."""
    if len(result.points_px) < 2:
        return False
    _count, labels = cv2.connectedComponents(result.mask, connectivity=8)
    label = _component_label_near_point(labels, result)
    if label == 0:
        return False

    component_y, component_x = np.where(labels == label)
    pivot_x, pivot_y_global = result.points_px[-2]
    pivot_y = pivot_y_global - result.roi_top
    band_height = max(1, result.band_boundaries_px[2] - result.band_boundaries_px[1])
    nearby = component_x[np.abs(component_y - pivot_y) <= max(12, band_height // 3)]
    if nearby.size == 0:
        return False
    right_extent = int(np.max(nearby)) - pivot_x
    left_extent = pivot_x - int(np.min(nearby))
    minimum_width = int(round(result.mask.shape[1] * minimum_width_ratio))
    return right_extent >= minimum_width and right_extent > left_extent * 1.25


def render_decision(
    frame,
    decision: PlannerDecision,
    *,
    right_branch_detected: bool,
    motor_action: str,
    ground_offset: float | None,
):
    color = {
        "STRAIGHT": (0, 220, 0),
        "TURN_RIGHT": (0, 165, 255),
        "STOP": (0, 0, 255),
    }[decision.intent.value]
    output = frame.copy()
    cv2.rectangle(output, (10, 76), (620, 222), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"ROUTE: {decision.intent.value}", (18, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.78, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"{decision.state.value}: {decision.reason}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    branch_text = "RIGHT BRANCH: DETECTED" if right_branch_detected else "RIGHT BRANCH: not detected"
    branch_color = (0, 165, 255) if right_branch_detected else (170, 170, 170)
    cv2.putText(output, branch_text, (18, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.52, branch_color, 1, cv2.LINE_AA)
    motor_color = (100, 220, 255) if motor_action != "DISPLAY_ONLY" else (170, 170, 170)
    cv2.putText(output, f"MOTOR: {motor_action}", (18, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.52, motor_color, 1, cv2.LINE_AA)
    ground_text = "GROUND OFFSET: unavailable" if ground_offset is None else f"GROUND OFFSET: {ground_offset:+.3f}"
    cv2.putText(output, ground_text, (18, 211), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 220, 100), 1, cv2.LINE_AA)
    return output


def emit(
    frame_index: int,
    result,
    decision: PlannerDecision,
    *,
    control_observation,
    right_branch_detected: bool,
    motor_action: str,
) -> None:
    print(
        json.dumps(
            {
                "frame_index": frame_index,
                "observation": result.observation.as_dict(),
                "ground_control_offset": control_observation.offset,
                "ground_control_heading": control_observation.heading,
                "route_intent": decision.intent.value,
                "route_state": decision.state.value,
                "reason": decision.reason,
                "right_branch_detected": right_branch_detected,
                "motor_action": motor_action,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ),
        flush=True,
    )


def main() -> int:
    args = parse_args()
    if args.process_fps < 0 or args.max_frames < 0 or not 0 <= args.debug_web_port <= 65535:
        raise ValueError("--process-fps, --max-frames, or --debug-web-port is invalid")

    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    planner = ClockwiseRectanglePlanner(planner_config_for_processing_rate(args.process_fps))
    ground_filter = GroundPlaneLineFilter()
    debug_web = None
    motor_executor = None
    if args.enable_motors:
        motor_executor = RobotWebMotorExecutor(MotorControlConfig(args.controller_url))
        motor_executor.configure()
        print(
            "motor_control=armed "
            f"straight_pwm={motor_executor.config.straight_pwm} "
            f"launch_pwm={motor_executor.config.launch_pwm} "
            f"launch_duration_s={motor_executor.config.launch_duration_seconds:.2f} "
            f"pivot_pwm={motor_executor.config.pivot_pwm} "
            f"curve_outer_pwm={motor_executor.config.curve_outer_pwm} "
            f"curve_inner_pwm={motor_executor.config.curve_inner_pwm}",
            flush=True,
        )
    print(
        "vision_control="
        f"process_fps={args.process_fps:g} "
        f"corner_approach_frames={planner.config.missing_before_turn} "
        f"recovery_forward_frames={planner.config.recovery_forward_frames} "
        f"max_turn_frames={planner.config.max_turn_frames}",
        flush=True,
    )
    capture = cv2.VideoCapture(source_value(args.source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")
    if args.debug_web_port:
        debug_web = DebugMjpegPublisher(args.debug_web_port)
        print(f"debug_web=http://0.0.0.0:{args.debug_web_port}", flush=True)

    minimum_interval = 0.0 if args.process_fps == 0 else 1.0 / args.process_fps
    last_processed_at = 0.0
    analysed_frames = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("camera stream ended before a usable frame arrived")

            now = time.monotonic()
            if now - last_processed_at < minimum_interval:
                continue
            last_processed_at = now

            detector_frame, corridor_polygon = track_corridor(
                frame,
                roi_top_ratio=detector.config.roi_top_ratio,
                top_width=args.track_roi_top_width,
                bottom_width=args.track_roi_bottom_width,
            )
            result = detector.detect(detector_frame, frame_index=analysed_frames, timestamp_ns=time.monotonic_ns())
            result = keep_near_connected_points(result, detector_frame.shape)
            ground_offset, ground_heading = ground_filter.update(
                line_lost=result.observation.line_lost,
                points_px=result.points_px,
                corridor_polygon=corridor_polygon,
            )
            control_observation = result.observation
            if ground_offset is not None and ground_heading is not None:
                control_observation = replace(
                    control_observation,
                    offset=ground_offset,
                    heading=ground_heading,
                )
            right_branch = has_connected_right_branch(result)
            new_line_candidate = (
                not control_observation.line_lost
                and control_observation.valid_bands >= 2
                and control_observation.confidence >= planner.config.minimum_confidence
                and not right_branch
            )
            new_line_ready = (
                new_line_candidate
                and control_observation.heading is not None
                and abs(control_observation.heading) <= 0.18
            )
            decision = planner.step(
                control_observation,
                right_corner_ahead=right_branch,
                new_line_candidate=new_line_candidate,
                new_line_ready=new_line_ready,
            )
            motor_action = motor_executor.apply(decision, control_observation) if motor_executor else "DISPLAY_ONLY"
            emit(
                analysed_frames,
                result,
                decision,
                control_observation=control_observation,
                right_branch_detected=right_branch,
                motor_action=motor_action,
            )
            annotated = render_decision(
                draw_track_corridor(render_debug(detector_frame, result), corridor_polygon),
                decision,
                right_branch_detected=right_branch,
                motor_action=motor_action,
                ground_offset=ground_offset,
            )
            if debug_web:
                debug_web.publish(
                    annotated,
                    {
                        "frame_index": analysed_frames,
                        "route_intent": decision.intent.value,
                        "route_state": decision.state.value,
                        "reason": decision.reason,
                        "motor_action": motor_action,
                        "ground_offset": ground_offset,
                    },
                )
            analysed_frames += 1

            if not args.headless:
                cv2.imshow("rectangle line monitor (Q/Esc to quit)", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            if args.max_frames and analysed_frames >= args.max_frames:
                break
    finally:
        capture.release()
        if motor_executor is not None:
            motor_executor.stop()
        if debug_web is not None:
            debug_web.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
