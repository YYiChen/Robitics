"""Display or drive any continuously visible taped path with lookahead P control."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import cv2


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[2]
TRACK_LINE_SRC = REPOSITORY_ROOT / "third_party" / "DeskMate-Advance" / "src"
if not TRACK_LINE_SRC.is_dir():
    raise RuntimeError(f"track_line source is missing: {TRACK_LINE_SRC}")
sys.path[:0] = [str(TRACK_LINE_SRC), str(REPOSITORY_ROOT)]

from continuous_motor_control import ContinuousMotorConfig, ContinuousMotorExecutor  # noqa: E402
from continuous_path_planner import ContinuousPathConfig, ContinuousPathPlanner  # noqa: E402
from marker_counter import MarkerCounter, MarkerCounterConfig  # noqa: E402
from debug_web import DebugMjpegPublisher  # noqa: E402
from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402
from tuning import (  # noqa: E402
    DEBUG_WEB_PORT,
    HEADING_WEIGHT,
    LAUNCH_PWM,
    LINE_LOST_PREDICTION_SECONDS,
    LINE_LOST_STOP_FRAMES,
    LINE_LOST_STOP_SECONDS,
    LOOKAHEAD_GAIN,
    MAXIMUM_CORRECTION_PWM,
    MINIMUM_CORRECTION_PWM,
    MINIMUM_WHEEL_PWM,
    MARKER_CLEAR_FRAMES,
    MARKER_CONFIRM_FRAMES,
    MARKERS_PER_LAP,
    PROCESS_FPS,
    STRAIGHT_PWM,
    SHARP_TURN_CORRECTION_PWM,
    SHARP_TURN_ERROR,
)


DEFAULT_CONFIG = TRACK_LINE_SRC / "track_line" / "config.dark_line.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://127.0.0.1:5000/video_feed")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--process-fps", type=float, default=PROCESS_FPS)
    parser.add_argument("--line-lost-stop-frames", type=int, default=LINE_LOST_STOP_FRAMES)
    parser.add_argument("--line-lost-prediction-seconds", type=float, default=LINE_LOST_PREDICTION_SECONDS)
    parser.add_argument("--line-lost-stop-seconds", type=float, default=LINE_LOST_STOP_SECONDS)
    parser.add_argument("--straight-pwm", type=int, default=STRAIGHT_PWM)
    parser.add_argument("--launch-pwm", type=int, default=LAUNCH_PWM)
    parser.add_argument("--lookahead-gain", type=float, default=LOOKAHEAD_GAIN)
    parser.add_argument("--heading-weight", type=float, default=HEADING_WEIGHT)
    parser.add_argument("--minimum-correction-pwm", type=int, default=MINIMUM_CORRECTION_PWM)
    parser.add_argument("--maximum-correction-pwm", type=int, default=MAXIMUM_CORRECTION_PWM)
    parser.add_argument("--minimum-wheel-pwm", type=int, default=MINIMUM_WHEEL_PWM)
    parser.add_argument("--sharp-turn-error", type=float, default=SHARP_TURN_ERROR)
    parser.add_argument("--sharp-turn-correction-pwm", type=int, default=SHARP_TURN_CORRECTION_PWM)
    parser.add_argument("--debug-web-port", type=int, default=DEBUG_WEB_PORT)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def source_value(source: str) -> int | str:
    return int(source) if source.strip().lstrip("-").isdigit() else source.strip()


def overlay(frame, decision, motor_action: str, observation, marker_update):
    output = frame.copy()
    color = (0, 0, 255) if decision.intent.value == "STOP" else (0, 220, 0)
    cv2.rectangle(output, (10, 76), (630, 216), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"PATH: {decision.intent.value}", (18, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"{decision.reason} lost_frames={decision.lost_frames}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
    target = "n/a" if observation.lookahead_offset is None else f"{observation.lookahead_offset:+.3f}"
    cv2.putText(output, f"LOOKAHEAD OFFSET: {target}", (18, 157), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 220, 255), 1, cv2.LINE_AA)
    cv2.putText(output, f"MOTOR: {motor_action}", (18, 181), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 220, 255), 1, cv2.LINE_AA)
    marker_state = "PASS" if marker_update.event else marker_update.state.value
    marker_color = (0, 220, 255) if marker_update.detected else (180, 180, 180)
    cv2.putText(
        output,
        f"MARKER: {marker_state}  {marker_update.marker_in_lap}/{MARKERS_PER_LAP}  LAP={marker_update.lap_count}",
        (18, 205),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        marker_color,
        1,
        cv2.LINE_AA,
    )
    return output


def main() -> int:
    args = parse_args()
    if args.process_fps <= 0 or not 0 <= args.debug_web_port <= 65535:
        raise ValueError("invalid process or web port setting")
    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    planner = ContinuousPathPlanner(ContinuousPathConfig(
        line_lost_stop_frames=args.line_lost_stop_frames,
        line_lost_prediction_seconds=args.line_lost_prediction_seconds,
        line_lost_stop_seconds=args.line_lost_stop_seconds,
    ))
    marker_counter = MarkerCounter(MarkerCounterConfig(
        confirm_frames=MARKER_CONFIRM_FRAMES,
        clear_frames=MARKER_CLEAR_FRAMES,
        markers_per_lap=MARKERS_PER_LAP,
    ))
    executor = None
    if args.enable_motors:
        executor = ContinuousMotorExecutor(ContinuousMotorConfig(
            args.controller_url,
            straight_pwm=args.straight_pwm,
            launch_pwm=args.launch_pwm,
            lookahead_gain=args.lookahead_gain,
            heading_weight=args.heading_weight,
            minimum_correction_pwm=args.minimum_correction_pwm,
            maximum_correction_pwm=args.maximum_correction_pwm,
            minimum_wheel_pwm=args.minimum_wheel_pwm,
            sharp_turn_error=args.sharp_turn_error,
            sharp_turn_correction_pwm=args.sharp_turn_correction_pwm,
        ))
        executor.arm()
    capture = cv2.VideoCapture(source_value(args.source))
    # HTTP/MJPEG sources otherwise tend to retain an old frame in OpenCV's buffer.
    # Keep only the newest camera frame; this is especially important for a Pi preview.
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")
    debug_web = DebugMjpegPublisher(args.debug_web_port) if args.debug_web_port else None
    interval, last, frame_index = 1.0 / args.process_fps, 0.0, 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("camera stream ended before a usable frame arrived")
            now = time.monotonic()
            if now - last < interval:
                continue
            last = now
            result = detector.detect(frame, frame_index=frame_index, timestamp_ns=time.monotonic_ns())
            marker_update = marker_counter.update(result.observation.marker_detected)
            decision = planner.step(result.observation, now)
            motor_action = executor.apply(decision.intent, result.observation) if executor else "DISPLAY_ONLY"
            annotated = overlay(render_debug(frame, result), decision, motor_action, result.observation, marker_update)
            payload = {
                "frame": frame_index,
                "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "intent": decision.intent.value,
                "reason": decision.reason,
                "motor": motor_action,
                "offset": result.observation.offset,
                "heading": result.observation.heading,
                "lookahead_offset": result.observation.lookahead_offset,
                "confidence": result.observation.confidence,
                "marker_detected": marker_update.detected,
                "marker_event": marker_update.event,
                "marker_point_px": result.observation.marker_point_px,
                "marker_branch_count": result.observation.marker_branch_count,
                "marker_in_lap": marker_update.marker_in_lap,
                "lap_count": marker_update.lap_count,
            }
            print(json.dumps(payload, ensure_ascii=False), flush=True)
            if debug_web:
                debug_web.publish(annotated, payload)
            frame_index += 1
            if not args.headless:
                cv2.imshow("continuous lookahead path (Q/Esc stops)", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        capture.release()
        if executor:
            executor.stop()
        if debug_web:
            debug_web.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
