"""Known clockwise rectangle: follow, fixed approach, fixed 90-degree right turn."""
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
STRAIGHT_ROOT = REPOSITORY_ROOT / "pi_service" / "experiments" / "straight_line_stop_validation"
if not TRACK_LINE_SRC.is_dir():
    raise RuntimeError(f"track_line source is missing: {TRACK_LINE_SRC}")
sys.path[:0] = [str(STRAIGHT_ROOT), str(TRACK_LINE_SRC), str(REPOSITORY_ROOT)]

from debug_web import DebugMjpegPublisher  # noqa: E402
from fixed_rectangle_planner import FixedClockwiseRectanglePlanner, FixedRectangleConfig  # noqa: E402
from rectangle_motor_control import RectangleMotorConfig, RectangleMotorExecutor  # noqa: E402
from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402


DEFAULT_CONFIG = TRACK_LINE_SRC / "track_line" / "config.dark_line.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://127.0.0.1:5000/video_feed")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--process-fps", type=float, default=30.0)
    parser.add_argument("--line-lost-corner-frames", type=int, default=5)
    parser.add_argument("--corner-forward-seconds", type=float, default=0.20)
    parser.add_argument("--right-turn-seconds", type=float, default=0.30)
    parser.add_argument("--reacquire-frames", type=int, default=3)
    parser.add_argument("--reacquire-timeout-seconds", type=float, default=0.80)
    parser.add_argument("--corners-to-complete", type=int, default=4)
    parser.add_argument("--straight-pwm", type=int, default=75)
    parser.add_argument("--launch-pwm", type=int, default=155)
    parser.add_argument("--pivot-pwm", type=int, default=155)
    parser.add_argument("--correction-gain", type=float, default=200.0)
    parser.add_argument("--minimum-correction-pwm", type=int, default=20)
    parser.add_argument("--maximum-correction-pwm", type=int, default=100)
    parser.add_argument("--debug-web-port", type=int, default=0)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def source_value(source: str) -> int | str:
    return int(source) if source.strip().lstrip("-").isdigit() else source.strip()


def overlay(frame, decision, motor_action: str):
    output = frame.copy()
    color = (0, 0, 255) if decision.intent.value == "STOP" else (0, 165, 255) if decision.intent.value == "PIVOT_RIGHT" else (0, 220, 0)
    cv2.rectangle(output, (10, 76), (630, 190), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"RECTANGLE: {decision.intent.value}", (18, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"{decision.state.value}: {decision.reason}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(output, f"corner={decision.corner_count}/4 lost_frames={decision.lost_frames}", (18, 157), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 220, 100), 1, cv2.LINE_AA)
    cv2.putText(output, f"MOTOR: {motor_action}", (18, 181), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 220, 255), 1, cv2.LINE_AA)
    return output


def main() -> int:
    args = parse_args()
    if args.process_fps < 0 or not 0 <= args.debug_web_port <= 65535:
        raise ValueError("invalid process or web port setting")
    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    planner = FixedClockwiseRectanglePlanner(FixedRectangleConfig(
        line_lost_corner_frames=args.line_lost_corner_frames,
        corner_forward_seconds=args.corner_forward_seconds,
        right_turn_seconds=args.right_turn_seconds,
        reacquire_frames=args.reacquire_frames,
        reacquire_timeout_seconds=args.reacquire_timeout_seconds,
        corners_to_complete=args.corners_to_complete,
    ))
    executor = None
    if args.enable_motors:
        executor = RectangleMotorExecutor(RectangleMotorConfig(
            args.controller_url, straight_pwm=args.straight_pwm, launch_pwm=args.launch_pwm,
            pivot_pwm=args.pivot_pwm, correction_gain=args.correction_gain,
            minimum_correction_pwm=args.minimum_correction_pwm, maximum_correction_pwm=args.maximum_correction_pwm,
        ))
        executor.arm()
    capture = cv2.VideoCapture(source_value(args.source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")
    debug_web = DebugMjpegPublisher(args.debug_web_port) if args.debug_web_port else None
    if debug_web:
        print(f"debug_web=http://0.0.0.0:{args.debug_web_port}", flush=True)
    interval, last, frame_index = (0.0 if args.process_fps == 0 else 1.0 / args.process_fps), 0.0, 0
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
            decision = planner.step(result.observation, now)
            motor_action = executor.apply(decision.intent, result.observation) if executor else "DISPLAY_ONLY"
            print(json.dumps({"frame": frame_index, "intent": decision.intent.value, "state": decision.state.value, "reason": decision.reason, "corner": decision.corner_count, "motor": motor_action}, ensure_ascii=False), flush=True)
            annotated = overlay(render_debug(frame, result), decision, motor_action)
            if debug_web:
                debug_web.publish(annotated, {"intent": decision.intent.value, "state": decision.state.value, "reason": decision.reason, "corner": decision.corner_count, "motor": motor_action})
            frame_index += 1
            if not args.headless:
                cv2.imshow("fixed clockwise rectangle (Q/Esc stops)", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        capture.release()
        if executor is not None:
            executor.stop()
        if debug_web is not None:
            debug_web.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
