"""Preview or drive I-shaped straight-line 180-degree turnaround validation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import cv2

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
TRACK_SRC = ROOT / "third_party" / "DeskMate-Advance" / "src"
CONTINUOUS = ROOT / "pi_service" / "experiments" / "continuous_path_validation"
sys.path[:0] = [str(TRACK_SRC), str(ROOT), str(CONTINUOUS), str(HERE)]

from debug_web import DebugMjpegPublisher  # noqa: E402
from i_turnaround_logic import IShapeTurnaroundPlanner, RouteEvidence, TurnaroundConfig, TurnaroundState  # noqa: E402
from pi_service.robot_client import RobotClientConfig, RobotWebClient  # noqa: E402
from track_line.config import LineDetectorConfig  # noqa: E402
from track_line.detector import OpenCVLineDetector  # noqa: E402
from track_line.visualization import render_debug  # noqa: E402


DEFAULT_CONFIG = TRACK_SRC / "track_line" / "config.fixed_green_white_course.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://127.0.0.1:5000/video_feed")
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--process-fps", type=float, default=20.0)
    parser.add_argument("--debug-web-port", type=int, default=5057)
    parser.add_argument("--log", type=Path, default=ROOT / "pi_service" / "logs" / "i_turnaround_validation" / "latest.jsonl")
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--straight-pwm", type=int, default=80)
    parser.add_argument("--pivot-pwm", type=int, default=155)
    parser.add_argument("--turn-direction", choices=("right", "left"), default="right")
    parser.add_argument("--pivot-min-seconds", type=float, default=1.60)
    parser.add_argument("--pivot-max-seconds", type=float, default=5.00)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def evidence_from(result, frame_height: int) -> RouteEvidence:
    observation = result.observation
    return RouteEvidence(observation.confidence, observation.line_lost, result.lookahead_px, result.centerline_px, observation.marker_detected, observation.marker_point_px, observation.marker_branch_count, frame_height)


def draw(frame, result, decision, enabled: bool, motor: str) -> object:
    output = render_debug(frame, result)
    color = (0, 220, 0) if decision.state is TurnaroundState.FOLLOW_STRAIGHT else (0, 170, 255) if decision.state is TurnaroundState.PIVOT_180 else (0, 0, 255)
    cv2.rectangle(output, (10, 76), (850, 215), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"I-TURN: {decision.state.value}", (18, 106), cv2.FONT_HERSHEY_SIMPLEX, .72, color, 2, cv2.LINE_AA)
    cv2.putText(output, decision.reason, (18, 133), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(output, f"end={decision.end_frames} reacquire={decision.reacquire_frames} pivot={decision.pivot_elapsed_seconds if decision.pivot_elapsed_seconds is not None else 'n/a'}", (18, 159), cv2.FONT_HERSHEY_SIMPLEX, .46, (100, 220, 255), 1, cv2.LINE_AA)
    cv2.putText(output, f"MOTOR: {motor}  {'ARMED' if enabled else 'DISPLAY ONLY'}", (18, 186), cv2.FONT_HERSHEY_SIMPLEX, .50, (100, 220, 255), 1, cv2.LINE_AA)
    return output


def main() -> int:
    args = parse_args()
    if not TRACK_SRC.is_dir() or not args.config.is_file():
        raise RuntimeError(f"missing line-detector config: {args.config}")
    detector = OpenCVLineDetector(LineDetectorConfig.from_json(args.config))
    planner = IShapeTurnaroundPlanner(TurnaroundConfig(pivot_min_seconds=args.pivot_min_seconds, pivot_max_seconds=args.pivot_max_seconds))
    client = RobotWebClient(RobotClientConfig(args.controller_url)) if args.enable_motors else None
    if client:
        client.require_arduino_online(); client.stop()
    capture = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")
    publisher = DebugMjpegPublisher(args.debug_web_port) if args.debug_web_port else None
    args.log.parent.mkdir(parents=True, exist_ok=True)
    interval, last, frame_index = 1.0 / max(1.0, args.process_fps), 0.0, 0
    try:
        with args.log.open("a", encoding="utf-8") as log:
            while True:
                ok, frame = capture.read()
                if not ok or frame is None: raise RuntimeError("camera stream ended")
                now = time.monotonic()
                if now - last < interval: continue
                last = now
                result = detector.detect(frame, frame_index=frame_index, timestamp_ns=time.monotonic_ns())
                evidence = evidence_from(result, frame.shape[0])
                decision = planner.step(evidence, now)
                motor = "DISPLAY_ONLY"
                if client:
                    if decision.state is TurnaroundState.FOLLOW_STRAIGHT and evidence.valid_line and evidence.confidence >= .45:
                        right, left = client.send_drive_pwm(args.straight_pwm, args.straight_pwm); motor = f"STRAIGHT R={right} L={left}"
                    elif decision.state is TurnaroundState.PIVOT_180:
                        pair = (args.pivot_pwm, -args.pivot_pwm) if args.turn_direction == "right" else (-args.pivot_pwm, args.pivot_pwm)
                        right, left = client.send_drive_pwm(*pair); motor = f"PIVOT_{args.turn_direction.upper()} R={right} L={left}"
                    else:
                        client.stop(); motor = "STOP"
                payload = {"wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"), "frame": frame_index, "state": decision.state.value, "reason": decision.reason, "end_frames": decision.end_frames, "reacquire_frames": decision.reacquire_frames, "pivot_elapsed_seconds": decision.pivot_elapsed_seconds, "confidence": result.observation.confidence, "marker_detected": result.observation.marker_detected, "marker_branch_count": result.observation.marker_branch_count, "motor": motor}
                log.write(json.dumps(payload, ensure_ascii=False) + "\n"); log.flush()
                annotated = draw(frame, result, decision, bool(client), motor)
                if publisher: publisher.publish(annotated, payload)
                if not args.headless:
                    cv2.imshow("I-shaped turnaround validation (Q/Esc exits)", annotated)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")): break
                frame_index += 1
    finally:
        capture.release()
        if client:
            try: client.stop()
            except RuntimeError: pass
        if publisher: publisher.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
