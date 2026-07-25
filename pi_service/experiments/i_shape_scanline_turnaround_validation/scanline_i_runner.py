"""Isolated scanline-based I-route turnaround validation; main route is untouched."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import cv2

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CONTINUOUS = ROOT / "pi_service" / "experiments" / "continuous_path_validation"
sys.path[:0] = [str(ROOT), str(CONTINUOUS), str(HERE)]

from debug_web import DebugMjpegPublisher  # noqa: E402
from pi_service.robot_client import RobotClientConfig, RobotWebClient  # noqa: E402
from scanline_i_logic import IShapeScanlineAnalyzer, IShapeTurnaroundPlanner, TurnaroundConfig, TurnaroundState  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://127.0.0.1:5000/video_feed")
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--process-fps", type=float, default=20.0)
    parser.add_argument("--debug-web-port", type=int, default=5058)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--straight-pwm", type=int, default=120)
    parser.add_argument("--pivot-pwm", type=int, default=200)
    parser.add_argument("--pivot-min-seconds", type=float, default=2.5)
    parser.add_argument("--pivot-max-seconds", type=float, default=5.0)
    parser.add_argument("--status-hz", type=float, default=4.0)
    parser.add_argument("--log", type=Path, default=ROOT / "pi_service" / "logs" / "i_shape_scanline_turnaround" / "latest.jsonl")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def require_isolated_controller(client: RobotWebClient) -> dict:
    status = client.status()
    if not status.get("robot", {}).get("arduino_online"):
        raise RuntimeError("Arduino is not online; no motor command was sent")
    if status.get("autonomous", {}).get("available"):
        raise RuntimeError("main autonomous route is present; restart 5000 with ROBOT_ENABLE_AUTONOMOUS_ROUTE=0 before this isolated drive test")
    return status


def robot_snapshot(status: dict | None) -> dict:
    robot = status.get("robot", {}) if status else {}
    return {"status_motor_output": robot.get("motor_output"), "arduino_reply": robot.get("reply"), "arduino_online": robot.get("arduino_online"), "status_last_rx_age": robot.get("last_rx_age")}


def draw(frame, result, decision, motor: str):
    output = frame.copy()
    mask = cv2.cvtColor(result.component_mask, cv2.COLOR_GRAY2BGR)
    output = cv2.addWeighted(output, .72, mask, .28, 0)
    evidence = result.evidence
    for y, x, width in evidence.line_centers:
        cv2.circle(output, (int(x), y), 5, (0, 255, 0), -1)
    if evidence.endpoint_y is not None:
        cv2.line(output, (0, evidence.endpoint_y), (output.shape[1] - 1, evidence.endpoint_y), (0, 165, 255), 2)
    cv2.rectangle(output, (10, 10), (930, 132), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, f"SCANLINE I-TURN: {decision.state.value}", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 0), 2)
    cv2.putText(output, f"bar={evidence.endpoint_detected} y={evidence.endpoint_y} width={evidence.endpoint_width} normal={evidence.normal_tape_width}", (18, 65), cv2.FONT_HERSHEY_SIMPLEX, .47, (100, 220, 255), 1)
    cv2.putText(output, f"confidence={evidence.confidence:.2f} {decision.reason} {motor}", (18, 92), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1)
    cv2.putText(output, "This experiment ignores skeleton branches; Q/Esc stops.", (18, 117), cv2.FONT_HERSHEY_SIMPLEX, .42, (190, 190, 190), 1)
    return output


def main() -> int:
    args = parse_args()
    analyzer = IShapeScanlineAnalyzer()
    planner = IShapeTurnaroundPlanner(TurnaroundConfig(pivot_min_seconds=args.pivot_min_seconds, pivot_max_seconds=args.pivot_max_seconds))
    client = RobotWebClient(RobotClientConfig(args.controller_url)) if args.enable_motors else None
    latest_status = require_isolated_controller(client) if client else None
    if client:
        client.stop()
    capture = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open source: {args.source}")
    publisher = DebugMjpegPublisher(args.debug_web_port) if args.debug_web_port else None
    args.log.parent.mkdir(parents=True, exist_ok=True)
    interval, last, frame_index, next_status = 1.0 / max(1.0, args.process_fps), 0.0, 0, 0.0
    previous_gray, previous_center = None, None
    try:
        with args.log.open("a", encoding="utf-8") as log:
            while True:
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("camera stream ended")
                now = time.monotonic()
                if now - last < interval:
                    continue
                last = now
                result = analyzer.analyze(frame)
                evidence = result.evidence
                decision = planner.step(evidence, now)
                gray = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (96, 72))
                scene_change = None if previous_gray is None else float(cv2.absdiff(previous_gray, gray).mean())
                previous_gray = gray
                route_changed = previous_center is not None and evidence.line_center_x != previous_center
                previous_center = evidence.line_center_x
                requested = acknowledged = None
                motor = "DISPLAY_ONLY"
                if client:
                    if decision.state is TurnaroundState.FOLLOW_STRAIGHT and evidence.valid_line and not evidence.endpoint_detected:
                        requested = (args.straight_pwm, args.straight_pwm)
                        acknowledged = client.send_drive_pwm(*requested)
                        motor = f"STRAIGHT R={acknowledged[0]} L={acknowledged[1]}"
                    elif decision.state is TurnaroundState.PIVOT_180:
                        requested = (args.pivot_pwm, -args.pivot_pwm)
                        acknowledged = client.send_drive_pwm(*requested)
                        motor = f"PIVOT_RIGHT R={acknowledged[0]} L={acknowledged[1]}"
                    else:
                        client.stop(); motor = "STOP"
                    if now >= next_status:
                        latest_status = client.status()
                        next_status = now + 1.0 / max(1.0, args.status_hz)
                payload = {"wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"), "frame": frame_index, "vision_confidence": evidence.confidence, "route_state": decision.state.value, "route_line_center_x": evidence.line_center_x, "transverse_bar_detected": evidence.endpoint_detected, "transverse_bar_position_px": evidence.endpoint_y, "transverse_bar_width_px": evidence.endpoint_width, "requested_right_pwm": requested[0] if requested else 0, "requested_left_pwm": requested[1] if requested else 0, "acknowledged_right_pwm": acknowledged[0] if acknowledged else 0, "acknowledged_left_pwm": acknowledged[1] if acknowledged else 0, "scene_change_score": scene_change, "scene_motion_detected": scene_change is not None and scene_change >= 8.0, "route_position_changed": route_changed, "reason": decision.reason, **robot_snapshot(latest_status)}
                log.write(json.dumps(payload, ensure_ascii=False) + "\n"); log.flush()
                annotated = draw(frame, result, decision, motor)
                if publisher:
                    publisher.publish(annotated, payload)
                if not args.headless:
                    cv2.imshow("Scanline I-shape turnaround (Q/Esc exits)", annotated)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break
                frame_index += 1
    finally:
        capture.release()
        if client:
            try:
                client.stop()
            except RuntimeError:
                pass
        if publisher:
            publisher.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
