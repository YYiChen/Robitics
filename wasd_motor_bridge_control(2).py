#!/usr/bin/env python3
"""Terminal controller for the Raspberry Pi -> Arduino motor bridge.

Compatible Arduino serial protocol:
    M,m1,m2,m3,m4\\n          raw PWM
    V,leftPPS,rightPPS\\n     speed-control targets (pulses/sec)
    KP,value  /  KI,value  /  KD,value

Motor order:
    M1 = right front     M2 = left front
    M3 = left rear       M4 = right rear

Controls:
    W / Up       Forward
    S / Down     Backward
    A / Left     Pivot left
    D / Right    Pivot right
    Q            Forward-left
    E            Forward-right
    Z            Backward-left
    C            Backward-right
    Space / K    Stop
    Tab          Toggle speed-control mode
    + / =        Increase PWM / target speed
    - / _        Decrease PWM / target speed
    ] / [        Adjust curve inner PWM
    X            Stop and exit

This is a latched terminal controller: one key selects an action and that
action continues until another action or Stop is selected.
"""

from __future__ import annotations

import argparse
import curses
import sys
import time
from dataclasses import dataclass

import serial
from serial import SerialException


BAUDRATE = 9600
HEARTBEAT_INTERVAL = 0.20
ARDUINO_STARTUP_DELAY = 2.5
PWM_STEP = 5
PWM_MIN = 0
PWM_MAX = 255
SPEED_STEP = 5.0       # pps per +/- key press
SPEED_DEFAULT = 30.0   # default target speed (pps)
SPEED_MAX = 200.0


@dataclass
class PwmSettings:
    straight: int = 60
    pivot: int = 150
    curve_outer: int = 160
    curve_inner: int = 60

    def clamp_all(self) -> None:
        self.straight = clamp_pwm(self.straight)
        self.pivot = clamp_pwm(self.pivot)
        self.curve_outer = clamp_pwm(self.curve_outer)
        self.curve_inner = clamp_pwm(self.curve_inner)


ACTION_NAMES = {
    "F": "前进",
    "B": "后退",
    "PL": "原地左转",
    "PR": "原地右转",
    "FL": "前进左转",
    "FR": "前进右转",
    "BL": "后退左转",
    "BR": "后退右转",
    "STOP": "停止",
}


def clamp_pwm(value: int) -> int:
    return max(PWM_MIN, min(PWM_MAX, int(value)))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过终端控制 Raspberry Pi + Arduino 四电机小车"
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Arduino 串口，默认 /dev/ttyACM0",
    )
    parser.add_argument("--straight", type=int, default=60)
    parser.add_argument("--pivot", type=int, default=150)
    parser.add_argument("--curve-outer", type=int, default=160)
    parser.add_argument("--curve-inner", type=int, default=60)
    return parser.parse_args()


def action_to_motors(
    action: str,
    pwm: PwmSettings,
) -> tuple[int, int, int, int]:
    """Return signed PWM values in M1, M2, M3, M4 order."""

    s = pwm.straight
    p = pwm.pivot
    outer = pwm.curve_outer
    inner = pwm.curve_inner

    mapping = {
        "F": (s, s, s, s),
        "B": (-s, -s, -s, -s),
        # M1/M4 are right side; M2/M3 are left side.
        "PL": (p, -p, -p, p),
        "PR": (-p, p, p, -p),
        "FL": (outer, inner, inner, outer),
        "FR": (inner, outer, outer, inner),
        "BL": (-inner, -outer, -outer, -inner),
        "BR": (-outer, -inner, -inner, -outer),
        "STOP": (0, 0, 0, 0),
    }
    return mapping[action]


def send_motors(
    ser: serial.Serial,
    motors: tuple[int, int, int, int],
) -> None:
    m1, m2, m3, m4 = motors
    payload = f"M,{m1},{m2},{m3},{m4}\n".encode("ascii")
    ser.write(payload)
    ser.flush()


def safe_stop(ser: serial.Serial) -> None:
    for _ in range(4):
        try:
            send_motors(ser, (0, 0, 0, 0))
            time.sleep(0.05)
        except SerialException:
            break


def send_speed_target(
    ser: serial.Serial,
    left_pps: float,
    right_pps: float,
) -> None:
    """Send a V command for rear-wheel speed control."""
    payload = f"V,{left_pps:.1f},{right_pps:.1f}\n".encode("ascii")
    ser.write(payload)
    ser.flush()


def send_pid_gain(ser: serial.Serial, gain: str, value: float) -> None:
    """Send a PID gain command: KP,val  /  KI,val  /  KD,val"""
    payload = f"{gain},{value:.3f}\n".encode("ascii")
    ser.write(payload)
    ser.flush()


def query_imu(ser: serial.Serial) -> None:
    """Send an IMU query to Arduino (fire-and-forget)."""
    try:
        ser.write(b"IMU\n")
        ser.flush()
    except SerialException:
        pass


def query_spd(ser: serial.Serial) -> None:
    """Send an SPD query to Arduino."""
    try:
        ser.write(b"SPD\n")
        ser.flush()
    except SerialException:
        pass


def read_replies(
    ser: serial.Serial,
    previous: str,
) -> tuple[str, tuple[float, float, float] | None, dict[str, float] | None]:
    """Read all pending serial lines.

    Returns ``(latest_reply, imu, spd)``.
      - ``imu``  — ``(roll, pitch, yaw)`` or ``None``
      - ``spd``  — speed dict or ``None``
    """
    latest = previous
    imu: tuple[float, float, float] | None = None
    spd: dict[str, float] | None = None

    while ser.in_waiting:
        line = ser.readline().decode(
            "utf-8",
            errors="replace",
        ).strip()
        if not line:
            continue

        # Parse IMU data: "IMU,roll,pitch,yaw"
        if line.startswith("IMU,"):
            parts = line.split(",")
            if len(parts) == 4:
                try:
                    imu = (
                        float(parts[1]),
                        float(parts[2]),
                        float(parts[3]),
                    )
                except ValueError:
                    pass
            continue

        # Parse SPD data: "SPD,curL,curR,tgtL,tgtR,pidL,pidR"
        if line.startswith("SPD,"):
            parts = line.split(",")
            if len(parts) == 7:
                try:
                    spd = {
                        "curL": float(parts[1]),
                        "curR": float(parts[2]),
                        "tgtL": float(parts[3]),
                        "tgtR": float(parts[4]),
                        "pidL": int(parts[5]),
                        "pidR": int(parts[6]),
                    }
                except ValueError:
                    pass
            continue

        latest = line

    return latest, imu, spd


def adjust_main_pwm(
    action: str,
    pwm: PwmSettings,
    delta: int,
) -> str:
    """Adjust the PWM category used by the currently selected action."""

    if action in ("F", "B", "STOP"):
        pwm.straight = clamp_pwm(pwm.straight + delta)
        return "直行 PWM"

    if action in ("PL", "PR"):
        pwm.pivot = clamp_pwm(pwm.pivot + delta)
        return "原地转向 PWM"

    pwm.curve_outer = clamp_pwm(pwm.curve_outer + delta)
    return "行进转弯外侧 PWM"


def draw_screen(
    screen: curses.window,
    action: str,
    pwm: PwmSettings,
    motors: tuple[int, int, int, int],
    port: str,
    last_reply: str,
    last_adjustment: str,
    imu: tuple[float, float, float] | None,
    speed_mode: bool,
    target_speed: float,
    spd: dict[str, float] | None,
    pid_gains: tuple[float, float, float],
) -> None:
    screen.erase()

    # IMU line
    if imu is not None:
        imu_line = f"IMU  Roll={imu[0]:7.2f}°  Pitch={imu[1]:7.2f}°  Yaw={imu[2]:7.2f}°"
    else:
        imu_line = "IMU  等待数据……"

    # Speed line
    if spd is not None:
        spd_line = (
            f"SPD  L={spd['curL']:6.1f}/{spd['tgtL']:6.1f} pps"
            f"  R={spd['curR']:6.1f}/{spd['tgtR']:6.1f} pps"
            f"  PID=[{spd['pidL']:>4},{spd['pidR']:>4}]"
        )
    else:
        spd_line = "SPD  等待数据……"

    # Mode line
    if speed_mode:
        mode_line = f"模式：速度控制  |  目标速度={target_speed:.0f} pps  |  "
        mode_line += f"Kp={pid_gains[0]:.2f} Ki={pid_gains[1]:.2f} Kd={pid_gains[2]:.2f}"
    else:
        mode_line = "模式：直接 PWM"

    lines = [
        "Arduino 四电机终端遥控",
        "",
        f"串口：{port}",
        mode_line,
        f"当前动作：{ACTION_NAMES[action]} ({action})",
        f"当前输出：M1={motors[0]:>4}  M2={motors[1]:>4}  "
        f"M3={motors[2]:>4}  M4={motors[3]:>4}",
        f"Arduino：{last_reply or '等待回执'}",
        imu_line,
        spd_line,
        "",
        f"直行 PWM：{pwm.straight}",
        f"原地转向 PWM：{pwm.pivot}",
        f"行进转弯外侧 PWM：{pwm.curve_outer}",
        f"行进转弯内侧 PWM：{pwm.curve_inner}",
        f"最近调整：{last_adjustment or '无'}",
        "",
        "W/↑ 前进       S/↓ 后退",
        "A/← 原地左转   D/→ 原地右转",
        "Q 前进左转     E 前进右转",
        "Z 后退左转     C 后退右转",
        "",
        "Tab：切换速度控制模式    空格/K：停车",
        "+/-：调整 PWM / 目标速度   [/]：调整内侧轮 PWM",
        "1/2/3：选择 Kp/Ki/Kd    ←→：调整 PID 增益",
        "X：停车并退出",
        "",
        "动作会保持，直到按下另一个动作键或停车键。",
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


def speed_action_to_targets(
    action: str,
    target_speed: float,
    pwm: PwmSettings,
) -> tuple[float, float, int, int]:
    """Return (left_pps, right_pps, front_left_PWM, front_right_PWM).

    In speed-control mode the rear wheels are driven by the V command
    (PID loop on Arduino) and the front wheels by a conventional M
    command for steering.
    """
    t = target_speed
    half = t * 0.5

    mapping: dict[str, tuple[float, float, int, int]] = {
        "F":    ( t,     t,     0,              0),
        "B":    (-t,    -t,     0,              0),
        "PL":   (-t,     t,    -pwm.pivot,      pwm.pivot),
        "PR":   ( t,    -t,     pwm.pivot,     -pwm.pivot),
        "FL":   ( half,  t,     pwm.curve_inner, pwm.curve_outer),
        "FR":   ( t,     half,  pwm.curve_outer, pwm.curve_inner),
        "BL":   (-t,    -half, -pwm.curve_inner,-pwm.curve_outer),
        "BR":   (-half, -t,    -pwm.curve_outer,-pwm.curve_inner),
        "STOP": ( 0.0,   0.0,   0,              0),
    }
    return mapping[action]


def controller(
    screen: curses.window,
    ser: serial.Serial,
    args: argparse.Namespace,
) -> None:
    curses.curs_set(0)
    screen.nodelay(True)
    screen.keypad(True)
    screen.timeout(30)

    pwm = PwmSettings(
        straight=args.straight,
        pivot=args.pivot,
        curve_outer=args.curve_outer,
        curve_inner=args.curve_inner,
    )
    pwm.clamp_all()

    # ---- speed-control state ----
    speed_mode = False
    target_speed = SPEED_DEFAULT
    pid_gains: list[float] = [2.0, 0.8, 0.05]   # Kp, Ki, Kd
    pid_select = 0   # 0=Kp, 1=Ki, 2=Kd
    pid_step = 0.1

    action = "STOP"
    motors = action_to_motors(action, pwm)
    last_reply = ""
    last_adjustment = ""
    last_imu: tuple[float, float, float] | None = None
    last_spd: dict[str, float] | None = None
    next_send = 0.0
    next_imu_query = 0.0
    next_spd_query = 0.0
    needs_redraw = True

    key_map = {
        ord("w"): "F",         ord("W"): "F",
        curses.KEY_UP: "F",
        ord("s"): "B",         ord("S"): "B",
        curses.KEY_DOWN: "B",
        ord("a"): "PL",        ord("A"): "PL",
        curses.KEY_LEFT: "PL",
        ord("d"): "PR",        ord("D"): "PR",
        curses.KEY_RIGHT: "PR",
        ord("q"): "FL",        ord("Q"): "FL",
        ord("e"): "FR",        ord("E"): "FR",
        ord("z"): "BL",        ord("Z"): "BL",
        ord("c"): "BR",        ord("C"): "BR",
        ord(" "): "STOP",      ord("k"): "STOP",     ord("K"): "STOP",
    }

    try:
        while True:
            now = time.monotonic()
            key = screen.getch()

            if key in (ord("x"), ord("X")):
                break

            # ---- mode toggle ----
            if key == 9:  # Tab
                speed_mode = not speed_mode
                action = "STOP"
                motors = action_to_motors(action, pwm)
                send_motors(ser, motors)
                # Disable speed control on Arduino when leaving speed mode.
                if not speed_mode:
                    send_speed_target(ser, 0.0, 0.0)
                next_send = now + HEARTBEAT_INTERVAL
                needs_redraw = True

            # ---- PID gain selection (1 / 2 / 3) ----
            elif key in (ord("1"), ord("2"), ord("3")):
                pid_select = key - ord("1")  # 0, 1, 2
                gain_names = ["Kp", "Ki", "Kd"]
                last_adjustment = f"选中 PID 参数：{gain_names[pid_select]}"
                needs_redraw = True

            # ---- PID gain adjustment (left / right arrows) ----
            elif key == curses.KEY_LEFT:
                pid_gains[pid_select] = max(0.0, pid_gains[pid_select] - pid_step)
                send_pid_gain(ser, ["KP", "KI", "KD"][pid_select],
                              pid_gains[pid_select])
                gain_names = ["Kp", "Ki", "Kd"]
                last_adjustment = (
                    f"{gain_names[pid_select]} → {pid_gains[pid_select]:.2f}"
                )
                needs_redraw = True

            elif key == curses.KEY_RIGHT:
                pid_gains[pid_select] += pid_step
                send_pid_gain(ser, ["KP", "KI", "KD"][pid_select],
                              pid_gains[pid_select])
                gain_names = ["Kp", "Ki", "Kd"]
                last_adjustment = (
                    f"{gain_names[pid_select]} → {pid_gains[pid_select]:.2f}"
                )
                needs_redraw = True

            # ---- action keys ----
            elif key in key_map:
                new_action = key_map[key]
                if new_action != action:
                    action = new_action
                    if speed_mode:
                        spd_l, spd_r, fl, fr = speed_action_to_targets(
                            action, target_speed, pwm
                        )
                        send_speed_target(ser, spd_l, spd_r)
                        send_motors(ser, (fl, fr, 0, 0))
                    else:
                        motors = action_to_motors(action, pwm)
                        send_motors(ser, motors)
                    next_send = now + HEARTBEAT_INTERVAL
                    needs_redraw = True

            # ---- +/- : PWM or speed adjustment ----
            elif key in (ord("+"), ord("=")):
                if speed_mode:
                    target_speed = min(SPEED_MAX, target_speed + SPEED_STEP)
                    last_adjustment = f"目标速度 → {target_speed:.0f} pps"
                    # Re-send with new target
                    spd_l, spd_r, fl, fr = speed_action_to_targets(
                        action, target_speed, pwm
                    )
                    send_speed_target(ser, spd_l, spd_r)
                else:
                    last_adjustment = adjust_main_pwm(action, pwm, PWM_STEP)
                    motors = action_to_motors(action, pwm)
                    send_motors(ser, motors)
                next_send = now + HEARTBEAT_INTERVAL
                needs_redraw = True

            elif key in (ord("-"), ord("_")):
                if speed_mode:
                    target_speed = max(0.0, target_speed - SPEED_STEP)
                    last_adjustment = f"目标速度 → {target_speed:.0f} pps"
                    spd_l, spd_r, fl, fr = speed_action_to_targets(
                        action, target_speed, pwm
                    )
                    send_speed_target(ser, spd_l, spd_r)
                else:
                    last_adjustment = adjust_main_pwm(action, pwm, -PWM_STEP)
                    motors = action_to_motors(action, pwm)
                    send_motors(ser, motors)
                next_send = now + HEARTBEAT_INTERVAL
                needs_redraw = True

            # ---- [ / ] : curve inner PWM (PWM mode only) ----
            elif key == ord("]") and not speed_mode:
                pwm.curve_inner = clamp_pwm(pwm.curve_inner + PWM_STEP)
                last_adjustment = "行进转弯内侧 PWM"
                motors = action_to_motors(action, pwm)
                send_motors(ser, motors)
                next_send = now + HEARTBEAT_INTERVAL
                needs_redraw = True

            elif key == ord("[") and not speed_mode:
                pwm.curve_inner = clamp_pwm(pwm.curve_inner - PWM_STEP)
                last_adjustment = "行进转弯内侧 PWM"
                motors = action_to_motors(action, pwm)
                send_motors(ser, motors)
                next_send = now + HEARTBEAT_INTERVAL
                needs_redraw = True

            # ---- heartbeat ----
            if now >= next_send:
                if speed_mode:
                    spd_l, spd_r, fl, fr = speed_action_to_targets(
                        action, target_speed, pwm
                    )
                    send_speed_target(ser, spd_l, spd_r)
                    send_motors(ser, (fl, fr, 0, 0))
                else:
                    motors = action_to_motors(action, pwm)
                    send_motors(ser, motors)
                next_send = now + HEARTBEAT_INTERVAL

            # ---- query IMU (5 Hz) ----
            if now >= next_imu_query:
                query_imu(ser)
                next_imu_query = now + 0.2

            # ---- query SPD (5 Hz) ----
            if now >= next_spd_query:
                query_spd(ser)
                next_spd_query = now + 0.2

            # ---- read replies ----
            new_reply, new_imu, new_spd = read_replies(ser, last_reply)
            if new_reply != last_reply:
                last_reply = new_reply
                needs_redraw = True
            if new_imu is not None:
                last_imu = new_imu
                needs_redraw = True
            if new_spd is not None:
                last_spd = new_spd
                needs_redraw = True

            if needs_redraw:
                motors_display = action_to_motors(action, pwm)
                draw_screen(
                    screen, action, pwm, motors_display,
                    args.port, last_reply, last_adjustment,
                    last_imu, speed_mode, target_speed,
                    last_spd, tuple(pid_gains),
                )
                needs_redraw = False

    finally:
        # Disable speed control before stopping.
        send_speed_target(ser, 0.0, 0.0)
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

            # Keep the latest startup message if the board sent one.
            startup_reply, _, _ = read_replies(ser, "")
            if startup_reply:
                print(f"Arduino：{startup_reply}")

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
