"""Live MJPEG monitor: OpenCV line detection plus clockwise-route intent.

The program is display-only. It never opens a serial port and never sends a
motor command. Press Q or Esc in the preview window to stop it.
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

from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402

from rectangle_route_planner import ClockwiseRectanglePlanner, PlannerDecision  # noqa: E402


DEFAULT_SOURCE = "http://100.80.46.54:5000/video_feed"
DEFAULT_CONFIG = TRACK_LINE_SRC / "track_line" / "config.dark_line.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=DEFAULT_SOURCE, help="MJPEG URL, video path, or camera index")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="OpenCV detector JSON config")
    parser.add_argument(
        "--process-fps",
        type=float,
        default=10.0,
        help="maximum frames per second to analyse; zero means analyse every frame",
    )
    parser.add_argument("--headless", action="store_true", help="print JSON without preview windows")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after N analysed frames; zero means run until Q/Esc")
    return parser.parse_args()


def source_value(source: str) -> int | str:
    source = source.strip()
    return int(source) if source.lstrip("-").isdigit() else source


def render_decision(frame, decision: PlannerDecision):
    color = {
        "STRAIGHT": (0, 220, 0),
        "TURN_RIGHT": (0, 165, 255),
        "STOP": (0, 0, 255),
    }[decision.intent.value]
    output = frame.copy()
    cv2.rectangle(output, (10, 76), (620, 143), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"ROUTE: {decision.intent.value}", (18, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.78, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"{decision.state.value}: {decision.reason}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def emit(frame_index: int, result, decision: PlannerDecision) -> None:
    print(
        json.dumps(
            {
                "frame_index": frame_index,
                "observation": result.observation.as_dict(),
                "route_intent": decision.intent.value,
                "route_state": decision.state.value,
                "reason": decision.reason,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ),
        flush=True,
    )


def main() -> int:
    args = parse_args()
    if args.process_fps < 0 or args.max_frames < 0:
        raise ValueError("--process-fps and --max-frames cannot be negative")

    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    planner = ClockwiseRectanglePlanner()
    capture = cv2.VideoCapture(source_value(args.source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")

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

            result = detector.detect(frame, frame_index=analysed_frames, timestamp_ns=time.monotonic_ns())
            decision = planner.step(result.observation)
            emit(analysed_frames, result, decision)
            annotated = render_decision(render_debug(frame, result), decision)
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
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
