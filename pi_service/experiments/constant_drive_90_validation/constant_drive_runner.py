"""Isolated constant-PWM drive test; it does not use camera, vision or card controls."""

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
    parser.add_argument("--pwm", type=int, default=90, help="Both drive wheels receive this positive PWM.")
    parser.add_argument("--command-hz", type=float, default=10.0)
    return parser.parse_args()


def validate(pwm: int, command_hz: float) -> None:
    if not 1 <= pwm <= 255:
        raise ValueError("pwm must be an integer in [1, 255]")
    if not 1 <= command_hz <= 30:
        raise ValueError("command_hz must be in [1, 30]")


def main() -> int:
    args = parse_args()
    validate(args.pwm, args.command_hz)
    client = RobotWebClient(RobotClientConfig(args.controller_url))
    client.require_arduino_online()
    interval = 1.0 / args.command_hz
    print(
        json.dumps(
            {"mode": "CONSTANT_DRIVE", "right_pwm": args.pwm, "left_pwm": args.pwm, "card_controls": "UNTOUCHED"},
            ensure_ascii=False,
        ),
        flush=True,
    )
    try:
        while True:
            right, left = client.send_drive_pwm(args.pwm, args.pwm)
            print(json.dumps({"right_pwm": right, "left_pwm": left}, ensure_ascii=False), flush=True)
            time.sleep(interval)
    finally:
        client.stop()
        print(json.dumps({"mode": "STOPPED"}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
