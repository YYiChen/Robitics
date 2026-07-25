"""Replay a recorded route video or JPG directory without touching the robot."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[2]
TRACK_LINE_SRC = REPOSITORY_ROOT / "third_party" / "DeskMate-Advance" / "src"
CONTINUOUS_DIR = REPOSITORY_ROOT / "pi_service" / "experiments" / "continuous_path_validation"
sys.path[:0] = [str(TRACK_LINE_SRC), str(CONTINUOUS_DIR)]

from node_planner import NodePlanner, NodePlannerConfig  # noqa: E402
from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402


DEFAULT_CONFIG = TRACK_LINE_SRC / "track_line" / "config.fixed_green_white_course.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="MP4/AVI file or directory of JPG/PNG frames")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--output-jsonl", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=10.0, help="Used for JPG directories and node timing")
    parser.add_argument("--node-stop-y-ratio", type=float, default=0.72)
    parser.add_argument("--node-hold-seconds", type=float, default=2.0)
    return parser.parse_args()


def iter_frames(source: Path):
    if source.is_dir():
        frames = sorted([*source.glob("*.jpg"), *source.glob("*.jpeg"), *source.glob("*.png")])
        if not frames:
            raise RuntimeError(f"no JPG/PNG frames in {source}")
        for path in frames:
            frame = cv2.imread(str(path))
            if frame is not None:
                yield path.name, frame
        return
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open replay source: {source}")
    index = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                break
            yield f"frame_{index:06d}", frame
            index += 1
    finally:
        capture.release()


def overlay_node(frame, decision, observation):
    output = frame.copy()
    color = (0, 0, 255) if decision.should_stop else (0, 220, 0)
    cv2.rectangle(output, (10, 76), (630, 168), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"NODE: {decision.state.value}", (18, 105), cv2.FONT_HERSHEY_SIMPLEX, .72, color, 2, cv2.LINE_AA)
    cv2.putText(output, f"{decision.reason}  next={decision.next_node}/4 lap={decision.lap_count}", (18, 132), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1, cv2.LINE_AA)
    y_ratio = "n/a" if observation.marker_y_ratio is None else f"{observation.marker_y_ratio:.2f}"
    cv2.putText(output, f"marker_y={y_ratio} simulated_command={'STOP' if decision.should_stop else 'FOLLOW'}", (18, 157), cv2.FONT_HERSHEY_SIMPLEX, .46, (100, 220, 255), 1, cv2.LINE_AA)
    return output


def main() -> int:
    args = parse_args()
    if args.fps <= 0:
        raise ValueError("fps must be positive")
    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    nodes = NodePlanner(NodePlannerConfig(stop_y_ratio=args.node_stop_y_ratio, hold_seconds=args.node_hold_seconds))
    args.output_video.parent.mkdir(parents=True, exist_ok=True)
    args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
    writer = None
    with args.output_jsonl.open("w", encoding="utf-8") as log:
        for index, (source_id, frame) in enumerate(iter_frames(args.source)):
            result = detector.detect(frame, frame_index=index)
            decision = nodes.step(
                marker_detected=result.observation.marker_detected,
                marker_y_ratio=result.observation.marker_y_ratio,
                now=index / args.fps,
            )
            annotated = overlay_node(render_debug(frame, result), decision, result.observation)
            if writer is None:
                height, width = annotated.shape[:2]
                writer = cv2.VideoWriter(str(args.output_video), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (width, height))
                if not writer.isOpened():
                    raise RuntimeError(f"cannot create output video: {args.output_video}")
            writer.write(annotated)
            row = {
                "frame": index, "source": source_id, "time_seconds": index / args.fps,
                "line": result.observation.as_dict(), "node_state": decision.state.value,
                "simulated_command": "STOP" if decision.should_stop else "FOLLOW",
                "node_reason": decision.reason, "next_node": decision.next_node,
                "completed_node": decision.completed_node, "lap_count": decision.lap_count,
            }
            log.write(json.dumps(row, ensure_ascii=False) + "\n")
    if writer is not None:
        writer.release()
    print(f"offline replay complete: {args.output_video}")
    print(f"frame log: {args.output_jsonl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
