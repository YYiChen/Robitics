"""Minimal line-following: steer gently while visible, stop when the line ends.

No turns, no corner recovery, and no route planning are implemented here.
Press Q/Esc in preview mode or Ctrl+C in headless mode to send STOP.
"""
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
sys.path.insert(0, str(TRACK_LINE_SRC))
sys.path.insert(0, str(REPOSITORY_ROOT))

from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402

from line_stop_planner import LineStopConfig, StraightLineStopPlanner  # noqa: E402
from straight_motor_control import StraightMotorConfig, StraightMotorExecutor  # noqa: E402


DEFAULT_CONFIG = TRACK_LINE_SRC / "track_line" / "config.dark_line.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://127.0.0.1:5000/video_feed")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--process-fps", type=float, default=30.0)
    parser.add_argument("--line-lost-stop-frames", type=int, default=5)
    parser.add_argument("--straight-pwm", type=int, default=65)
    parser.add_argument("--launch-pwm", type=int, default=155)
    parser.add_argument("--minimum-correction-pwm", type=int, default=20)
    parser.add_argument("--maximum-correction-pwm", type=int, default=60)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def source_value(source: str) -> int | str:
    source = source.strip()
    return int(source) if source.lstrip("-").isdigit() else source


def overlay(frame, decision, motor_action: str, observation):
    output = frame.copy()
    color = (0, 220, 0) if decision.intent.value == "STRAIGHT" else (0, 0, 255)
    cv2.rectangle(output, (10, 76), (620, 185), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"LINE MODE: {decision.intent.value}", (18, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.76, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"{decision.reason} lost_frames={decision.lost_frames}", (18, 133), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(output, f"MOTOR: {motor_action}", (18, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 220, 255), 1, cv2.LINE_AA)
    offset = "unavailable" if observation.offset is None else f"{observation.offset:+.3f}"
    cv2.putText(output, f"OFFSET: {offset}", (18, 182), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 220, 100), 1, cv2.LINE_AA)
    return output


def main() -> int:
    args = parse_args()
    if args.process_fps < 0:
        raise ValueError("--process-fps cannot be negative")
    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    planner = StraightLineStopPlanner(LineStopConfig(line_lost_stop_frames=args.line_lost_stop_frames))
    executor = None
    if args.enable_motors:
        executor = StraightMotorExecutor(
            StraightMotorConfig(
                args.controller_url,
                straight_pwm=args.straight_pwm,
                launch_pwm=args.launch_pwm,
                minimum_correction_pwm=args.minimum_correction_pwm,
                maximum_correction_pwm=args.maximum_correction_pwm,
            )
        )
        executor.arm()
    capture = cv2.VideoCapture(source_value(args.source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")

    minimum_interval = 0.0 if args.process_fps == 0 else 1.0 / args.process_fps
    last_processed_at = 0.0
    frame_index = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("camera stream ended before a usable frame arrived")
            now = time.monotonic()
            if now - last_processed_at < minimum_interval:
                continue
            last_processed_at = now

            result = detector.detect(frame, frame_index=frame_index, timestamp_ns=time.monotonic_ns())
            decision = planner.step(result.observation)
            motor_action = executor.apply(decision.intent, result.observation) if executor else "DISPLAY_ONLY"
            print(json.dumps({"frame": frame_index, "intent": decision.intent.value, "reason": decision.reason, "lost_frames": decision.lost_frames, "offset": result.observation.offset, "motor": motor_action}, ensure_ascii=False), flush=True)
            annotated = overlay(render_debug(frame, result), decision, motor_action, result.observation)
            frame_index += 1
            if not args.headless:
                cv2.imshow("straight line stop validation (Q/Esc stops)", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        capture.release()
        if executor is not None:
            executor.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
