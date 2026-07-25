"""Isolated left/right wheel mapping test; camera, vision and card motors are untouched."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT))

from pi_service.robot_client import RobotClientConfig, RobotWebClient  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-url", default="http://127.0.0.1:5000")
    parser.add_argument("--slow-pwm", type=int, default=55, help="PWM of the deliberately slower side.")
    parser.add_argument("--fast-pwm", type=int, default=180, help="PWM of the deliberately faster side.")
    parser.add_argument("--duration", type=float, default=5.0, help="Seconds for each test phase; capped at 10.0.")
    parser.add_argument("--command-hz", type=float, default=12.0, help="Drive command heartbeat frequency.")
    parser.add_argument("--pause-seconds", type=float, default=1.0, help="Stopped gap between the two phases.")
    parser.add_argument("--dry-run", action="store_true", help="Print the sequence without contacting Arduino.")
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not 1 <= args.slow_pwm <= 255 or not 1 <= args.fast_pwm <= 255:
        raise ValueError("PWM must be in [1, 255]")
    if args.fast_pwm <= args.slow_pwm:
        raise ValueError("fast-pwm must be greater than slow-pwm")
    if not .1 <= args.duration <= 10.0:
        raise ValueError("duration must be in [0.1, 10.0] seconds")
    if not 1 <= args.command_hz <= 30:
        raise ValueError("command-hz must be in [1, 30]")
    if not 0 <= args.pause_seconds <= 5:
        raise ValueError("pause-seconds must be in [0, 5]")


def phases(args: argparse.Namespace) -> tuple[tuple[str, int, int], ...]:
    return (
        ("RIGHT_SLOW_LEFT_FAST", args.slow_pwm, args.fast_pwm),
        ("RIGHT_FAST_LEFT_SLOW", args.fast_pwm, args.slow_pwm),
    )


def print_event(**payload: object) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def run_phase(client: RobotWebClient, name: str, right_pwm: int, left_pwm: int, args: argparse.Namespace) -> None:
    print_event(
        event="phase_start",
        phase=name,
        right_pwm=right_pwm,
        left_pwm=left_pwm,
        duration_seconds=args.duration,
        instruction="观察右侧和左侧轮组，确认哪一侧实际更快。",
    )
    until = time.monotonic() + args.duration
    interval = 1.0 / args.command_hz
    while time.monotonic() < until:
        right, left = client.send_drive_pwm(right_pwm, left_pwm)
        print_event(event="drive_ack", phase=name, right_pwm=right, left_pwm=left)
        time.sleep(interval)
    client.stop()
    print_event(event="phase_stop", phase=name)


def main() -> int:
    args = parse_args()
    validate(args)
    sequence = phases(args)
    print_event(mode="DIFFERENTIAL_DRIVE_MAPPING", phases=sequence, card_controls="UNTOUCHED")
    if args.dry_run:
        return 0
    client = RobotWebClient(RobotClientConfig(args.controller_url))
    client.require_arduino_online()
    try:
        for index, (name, right_pwm, left_pwm) in enumerate(sequence):
            run_phase(client, name, right_pwm, left_pwm, args)
            if index + 1 < len(sequence) and args.pause_seconds:
                print_event(event="pause", duration_seconds=args.pause_seconds)
                time.sleep(args.pause_seconds)
    finally:
        client.stop()
        print_event(event="final_stop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
