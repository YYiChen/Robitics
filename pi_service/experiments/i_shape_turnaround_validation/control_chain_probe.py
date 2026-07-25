"""M1/M2-only right-pivot probe with controller telemetry JSONL evidence.

This intentionally contains no vision or route logic.  It proves the command
path up to Arduino-reported output, while the required operator observation
proves the physical wheel and chassis motion.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(ROOT), str(HERE)]

from pi_service.robot_client import RobotClientConfig, RobotWebClient  # noqa: E402
from runtime_guard import require_no_competing_autonomous_route  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--right-pwm", type=int, default=200)
    parser.add_argument("--left-pwm", type=int, default=-200)
    parser.add_argument("--duration-seconds", type=float, default=3.0)
    parser.add_argument("--command-hz", type=float, default=12.0)
    parser.add_argument("--status-hz", type=float, default=4.0)
    parser.add_argument(
        "--log",
        type=Path,
        default=ROOT / "pi_service" / "logs" / "i_turnaround_validation" / "control_chain_probe_latest.jsonl",
    )
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not -255 <= args.right_pwm <= 255 or not -255 <= args.left_pwm <= 255:
        raise ValueError("right-pwm and left-pwm must be in [-255, 255]")
    if args.right_pwm <= 0 or args.left_pwm >= 0:
        raise ValueError("this right-pivot probe requires right-pwm > 0 and left-pwm < 0")
    if not 0.5 <= args.duration_seconds <= 10.0:
        raise ValueError("duration-seconds must be in [0.5, 10.0]")
    if not 1.0 <= args.command_hz <= 20.0 or not 1.0 <= args.status_hz <= 10.0:
        raise ValueError("command-hz must be in [1, 20] and status-hz in [1, 10]")


def robot_snapshot(status: dict[str, object] | None) -> dict[str, object]:
    robot = status.get("robot", {}) if status else {}
    return {
        "status_motor_output": robot.get("motor_output"),
        "arduino_reply": robot.get("reply"),
        "arduino_online": robot.get("arduino_online"),
        "status_last_rx_age": robot.get("last_rx_age"),
    }


def main() -> int:
    args = parse_args()
    validate(args)
    client = RobotWebClient(RobotClientConfig(args.controller_url))
    initial = require_no_competing_autonomous_route(client.status())
    args.log.parent.mkdir(parents=True, exist_ok=True)
    command_interval = 1.0 / args.command_hz
    status_interval = 1.0 / args.status_hz
    latest_status: dict[str, object] | None = initial
    next_status = 0.0
    deadline = time.monotonic() + args.duration_seconds
    sequence = 0
    try:
        with args.log.open("a", encoding="utf-8") as log:
            while time.monotonic() < deadline:
                requested = (args.right_pwm, args.left_pwm)
                acknowledged = client.send_drive_pwm(*requested)
                now = time.monotonic()
                if now >= next_status:
                    latest_status = client.status()
                    next_status = now + status_interval
                payload = {
                    "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                    "sequence": sequence,
                    "frame": None,
                    "vision_confidence": None,
                    "route_state": "CONTROL_CHAIN_RIGHT_PIVOT",
                    "transverse_bar_position_px": None,
                    "requested_right_pwm": requested[0],
                    "requested_left_pwm": requested[1],
                    "acknowledged_right_pwm": acknowledged[0],
                    "acknowledged_left_pwm": acknowledged[1],
                    "scene_motion_detected": None,
                    "route_position_changed": None,
                    "physical_motion_requires_operator_confirmation": True,
                    **robot_snapshot(latest_status),
                }
                log.write(json.dumps(payload, ensure_ascii=False) + "\n")
                log.flush()
                sequence += 1
                time.sleep(command_interval)
    finally:
        try:
            client.stop()
        except RuntimeError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
