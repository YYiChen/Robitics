#!/usr/bin/env python3
"""Raspberry Pi VNC/SSH terminal WASD controller for Arduino robot.

Controls
--------
W / Up       Forward
S / Down     Backward
A / Left     Forward-left
D / Right    Forward-right
Q            Pivot left
E            Pivot right
Z            Backward-left
C            Backward-right
Space / K    Stop
+ / =        Increase speed
- / _        Decrease speed
X            Stop and exit

The selected movement persists until another key is pressed.
A heartbeat is sent every 0.2 seconds so it works with the Arduino
communication timeout/watchdog.
"""

from __future__ import annotations

import argparse
import curses
import sys
import time

import serial
from serial import SerialException


BAUDRATE = 9600
HEARTBEAT_INTERVAL = 0.20
ARDUINO_STARTUP_DELAY = 2.5
SPEED_STEP = 10

ACTION_NAMES = {
    "F": "前进",
    "B": "后退",
    "L": "前进左转",
    "R": "前进右转",
    "Q": "原地左转",
    "E": "原地右转",
    "Z": "后退左转",
    "C": "后退右转",
    "S": "停止",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过终端 WASD 实时控制 QArduino 小车"
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Arduino 串口，默认 /dev/ttyACM0",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=220,
        help="初始 PWM 速度，默认 220",
    )
    parser.add_argument(
        "--min-speed",
        type=int,
        default=180,
        help="最低可调 PWM，默认 180",
    )
    return parser.parse_args()


def send_command(
    ser: serial.Serial,
    action: str,
    speed: int,
) -> None:
    payload = f"{action},{speed}\n".encode("ascii")
    ser.write(payload)
    ser.flush()


def safe_stop(ser: serial.Serial) -> None:
    for _ in range(4):
        try:
            send_command(ser, "S", 0)
            time.sleep(0.05)
        except SerialException:
            break


def draw_screen(
    screen: curses.window,
    action: str,
    speed: int,
    port: str,
    last_reply: str,
) -> None:
    screen.erase()

    lines = [
        "Arduino 小车实时遥控",
        "",
        f"串口：{port}",
        f"当前动作：{ACTION_NAMES[action]} ({action})",
        f"当前速度：{speed}",
        f"Arduino：{last_reply or '等待回执'}",
        "",
        "W/↑ 前进     S/↓ 后退",
        "A/← 左转     D/→ 右转",
        "Q 原地左转   E 原地右转",
        "Z 后退左转   C 后退右转",
        "",
        "空格/K：停车",
        "+/-：调整速度",
        "X：停车并退出",
        "",
        "动作会保持，直到按下另一个方向键或停车键。",
    ]

    height, width = screen.getmaxyx()

    for row, text in enumerate(lines):
        if row >= height - 1:
            break
        try:
            screen.addstr(row, 0, text[: max(0, width - 1)])
        except curses.error:
            pass

    screen.refresh()


def read_replies(ser: serial.Serial, previous: str) -> str:
    latest = previous

    while ser.in_waiting:
        line = ser.readline().decode(
            "utf-8",
            errors="replace",
        ).strip()
        if line:
            latest = line

    return latest


def controller(
    screen: curses.window,
    ser: serial.Serial,
    args: argparse.Namespace,
) -> None:
    curses.curs_set(0)
    screen.nodelay(True)
    screen.keypad(True)
    screen.timeout(30)

    min_speed = max(0, min(255, args.min_speed))
    speed = max(min_speed, min(255, args.speed))

    action = "S"
    last_reply = ""
    next_send = 0.0
    needs_redraw = True

    key_map = {
        ord("w"): "F",
        ord("W"): "F",
        curses.KEY_UP: "F",
        ord("s"): "B",
        ord("S"): "B",
        curses.KEY_DOWN: "B",
        ord("a"): "L",
        ord("A"): "L",
        curses.KEY_LEFT: "L",
        ord("d"): "R",
        ord("D"): "R",
        curses.KEY_RIGHT: "R",
        ord("q"): "Q",
        ord("Q"): "Q",
        ord("e"): "E",
        ord("E"): "E",
        ord("z"): "Z",
        ord("Z"): "Z",
        ord("c"): "C",
        ord("C"): "C",
        ord(" "): "S",
        ord("k"): "S",
        ord("K"): "S",
    }

    try:
        while True:
            now = time.monotonic()
            key = screen.getch()

            if key in (ord("x"), ord("X")):
                break

            if key in key_map:
                new_action = key_map[key]

                if new_action != action:
                    action = new_action
                    send_command(
                        ser,
                        action,
                        0 if action == "S" else speed,
                    )
                    next_send = now + HEARTBEAT_INTERVAL
                    needs_redraw = True

            elif key in (ord("+"), ord("=")):
                new_speed = min(255, speed + SPEED_STEP)

                if new_speed != speed:
                    speed = new_speed
                    if action != "S":
                        send_command(ser, action, speed)
                        next_send = now + HEARTBEAT_INTERVAL
                    needs_redraw = True

            elif key in (ord("-"), ord("_")):
                new_speed = max(min_speed, speed - SPEED_STEP)

                if new_speed != speed:
                    speed = new_speed
                    if action != "S":
                        send_command(ser, action, speed)
                        next_send = now + HEARTBEAT_INTERVAL
                    needs_redraw = True

            if now >= next_send:
                send_command(
                    ser,
                    action,
                    0 if action == "S" else speed,
                )
                next_send = now + HEARTBEAT_INTERVAL

            new_reply = read_replies(ser, last_reply)
            if new_reply != last_reply:
                last_reply = new_reply
                needs_redraw = True

            if needs_redraw:
                draw_screen(
                    screen,
                    action,
                    speed,
                    args.port,
                    last_reply,
                )
                needs_redraw = False

    finally:
        safe_stop(ser)


def main() -> int:
    args = parse_args()

    try:
        with serial.Serial(
            port=args.port,
            baudrate=BAUDRATE,
            timeout=0.05,
            write_timeout=1,
        ) as ser:
            print(f"正在连接 {args.port}……")
            time.sleep(ARDUINO_STARTUP_DELAY)
            ser.reset_input_buffer()

            curses.wrapper(controller, ser, args)

        print("已停车并退出。")
        return 0

    except SerialException as exc:
        print(f"串口错误：{exc}", file=sys.stderr)
        print(
            "请检查 Arduino 端口、USB 连接，以及是否有其他程序占用串口。",
            file=sys.stderr,
        )
        return 1
    except KeyboardInterrupt:
        print("\n已停车并退出。")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
