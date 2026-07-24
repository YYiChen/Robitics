#!/usr/bin/env python3
"""One-process web controller for a Raspberry Pi 5 + Arduino Mega robot.

Open http://<raspberry-pi-ip>:5000/ to use the camera, movement controls,
PWM/speed/PID settings, IMU/speed telemetry, snapshots, and emergency stop.

Compatible Arduino serial protocol:
    M,m1,m2,m3,m4\n          raw signed PWM
    V,leftPPS,rightPPS\n     rear-wheel speed targets
    KP,value / KI,value / KD,value
    IMU\n                     query; reply IMU,roll,pitch,yaw
    SPD\n                     query; reply SPD,curL,curR,tgtL,tgtR,pidL,pidR

Motor order retained from the existing project:
    M1 = right front, M2 = left front, M3 = left rear, M4 = right rear
"""

from __future__ import annotations

import argparse
import atexit
import glob
import json
import signal
import socket
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterator

import serial
from flask import Flask, Response, jsonify, request, send_from_directory
from serial import SerialException


BAUDRATE = 9600
SEND_INTERVAL_SECONDS = 0.20
CLIENT_TIMEOUT_SECONDS = 0.80
SERIAL_RECONNECT_SECONDS = 2.0
ARDUINO_STARTUP_DELAY_SECONDS = 2.5
QUERY_INTERVAL_SECONDS = 0.25
PWM_LIMIT = 255
SPEED_LIMIT = 200.0

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


@dataclass
class RobotConfig:
    straight_pwm: int = 60
    pivot_pwm: int = 150
    curve_outer_pwm: int = 160
    curve_inner_pwm: int = 60
    speed_mode: bool = False
    target_speed: float = 30.0
    kp: float = 2.0
    ki: float = 0.8
    kd: float = 0.05

    def normalise(self) -> "RobotConfig":
        self.straight_pwm = clamp_pwm(self.straight_pwm)
        self.pivot_pwm = clamp_pwm(self.pivot_pwm)
        self.curve_outer_pwm = clamp_pwm(self.curve_outer_pwm)
        self.curve_inner_pwm = clamp_pwm(self.curve_inner_pwm)
        self.target_speed = max(0.0, min(SPEED_LIMIT, float(self.target_speed)))
        self.kp = max(0.0, float(self.kp))
        self.ki = max(0.0, float(self.ki))
        self.kd = max(0.0, float(self.kd))
        self.speed_mode = bool(self.speed_mode)
        return self


def clamp_pwm(value: int | float) -> int:
    return max(0, min(PWM_LIMIT, int(value)))


def clamp_motor(value: int | float) -> int:
    return max(-PWM_LIMIT, min(PWM_LIMIT, int(value)))


def get_display_ip() -> str:
    """Prefer the Tailscale address, then fall back to a LAN address."""
    try:
        import subprocess

        result = subprocess.run(
            ["tailscale", "ip", "-4"],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
        address = result.stdout.strip().splitlines()
        if address:
            return address[0]
    except (OSError, subprocess.SubprocessError):
        pass

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return str(sock.getsockname()[0])
    except OSError:
        return "100.80.46.54"
    finally:
        sock.close()


class CameraStreamer:
    """Capture the latest CSI frame once and share it with all web clients."""

    def __init__(self, width: int, height: int, quality: int, fps: float) -> None:
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        self.quality = max(1, min(100, int(quality)))
        self.fps = max(1.0, float(fps))

        self._condition = threading.Condition()
        self._latest_jpeg: bytes | None = None
        self._sequence = 0
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._picam2 = None
        self._cv2 = None
        self.status = "未启动"
        self.error = ""

    @property
    def online(self) -> bool:
        return self.status == "运行中" and not self.error

    def start(self) -> bool:
        try:
            import cv2
            from picamera2 import Picamera2
        except ImportError as exc:
            self.status = "不可用"
            self.error = f"缺少依赖：{exc}"
            return False

        try:
            self.status = "正在初始化"
            self._cv2 = cv2
            picam2 = Picamera2()
            config = picam2.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls={
                    "FrameDurationLimits": (25000, 25000),
                    "AwbEnable": True,
                    "AeEnable": True,
                },
                buffer_count=4,
            )
            picam2.configure(config)
            picam2.start()
            self._picam2 = picam2
            self.status = "运行中"
            self._thread = threading.Thread(
                target=self._capture_loop,
                name="camera-capture",
                daemon=True,
            )
            self._thread.start()
            return True
        except Exception as exc:  # Hardware failures should not kill web control.
            self.status = "初始化失败"
            self.error = str(exc)
            return False

    def _capture_loop(self) -> None:
        interval = 1.0 / self.fps
        try:
            while not self._stop_event.is_set():
                started = time.monotonic()
                frame = self._picam2.capture_array()
                frame = self._cv2.cvtColor(frame, self._cv2.COLOR_RGB2BGR)
                ok, buffer = self._cv2.imencode(
                    ".jpg",
                    frame,
                    [self._cv2.IMWRITE_JPEG_QUALITY, self.quality],
                )
                if ok:
                    with self._condition:
                        self._latest_jpeg = buffer.tobytes()
                        self._sequence += 1
                        self._condition.notify_all()

                remaining = interval - (time.monotonic() - started)
                if remaining > 0:
                    self._stop_event.wait(remaining)
        except Exception as exc:
            self.status = "采集错误"
            self.error = str(exc)
            with self._condition:
                self._condition.notify_all()

    def iter_mjpeg(self) -> Iterator[bytes]:
        last_sequence = -1
        while not self._stop_event.is_set():
            with self._condition:
                self._condition.wait_for(
                    lambda: (
                        self._sequence != last_sequence
                        or self._stop_event.is_set()
                        or bool(self.error)
                    ),
                    timeout=1.0,
                )
                if self._stop_event.is_set() or self.error:
                    break
                frame = self._latest_jpeg
                last_sequence = self._sequence

            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )

    def save_snapshot(self, directory: Path) -> Path:
        with self._condition:
            frame = self._latest_jpeg
        if frame is None:
            raise RuntimeError("摄像头尚未产生可保存的画面")

        directory.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime("capture_%Y%m%d_%H%M%S_%f.jpg")
        path = directory / filename
        path.write_bytes(frame)
        return path

    def status_dict(self) -> dict[str, Any]:
        return {
            "online": self.online,
            "status": self.status,
            "error": self.error,
            "resolution": f"{self.width}x{self.height}",
            "fps": self.fps,
        }

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._picam2 is not None:
            try:
                self._picam2.stop()
            except Exception:
                pass
        self.status = "已关闭"


class RobotController:
    """Thread-safe serial controller driven by browser key heartbeats."""

    def __init__(self, requested_port: str, config_path: Path) -> None:
        self.requested_port = requested_port
        self.config_path = config_path
        self.lock = threading.RLock()
        self.serial_lock = threading.RLock()
        self.command_event = threading.Event()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None

        self.serial_port: serial.Serial | None = None
        self.actual_port = ""
        self.last_serial_attempt = 0.0
        self.last_serial_reply = ""
        self.last_serial_error = ""

        self.held_keys: set[str] = set()
        self.last_client_update = 0.0
        self.client_connected = False
        self.current_action = "STOP"
        self.last_sent = (0, 0, 0, 0)
        self.last_speed_target = (0.0, 0.0)
        self.last_mode_sent: bool | None = None

        self.imu: tuple[float, float, float] | None = None
        self.spd: dict[str, float] | None = None
        self.config = self._load_config()

    def _load_config(self) -> RobotConfig:
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
            return RobotConfig(**data).normalise()
        except (FileNotFoundError, json.JSONDecodeError, TypeError, ValueError):
            return RobotConfig()

    def _save_config(self) -> None:
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.config_path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(asdict(self.config), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary.replace(self.config_path)

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._control_loop,
            name="robot-control",
            daemon=True,
        )
        self.thread.start()

    def _candidate_ports(self) -> list[str]:
        candidates: list[str] = []
        if self.requested_port and self.requested_port.lower() != "auto":
            candidates.append(self.requested_port)
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
        candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
        # Preserve order while removing duplicates.
        return list(dict.fromkeys(candidates))

    @property
    def serial_connected(self) -> bool:
        return bool(self.serial_port is not None and self.serial_port.is_open)

    def _connect_serial_if_needed(self) -> None:
        if self.serial_connected:
            return
        now = time.monotonic()
        if now - self.last_serial_attempt < SERIAL_RECONNECT_SECONDS:
            return
        self.last_serial_attempt = now

        candidates = self._candidate_ports()
        if not candidates:
            self.last_serial_error = "没有发现 /dev/ttyACM* 或 /dev/ttyUSB* 串口"
            return

        errors: list[str] = []
        for port_name in candidates:
            try:
                port = serial.Serial(
                    port=port_name,
                    baudrate=BAUDRATE,
                    timeout=0.02,
                    write_timeout=0.5,
                )
                time.sleep(ARDUINO_STARTUP_DELAY_SECONDS)
                port.reset_input_buffer()
                self.serial_port = port
                self.actual_port = port_name
                self.last_serial_error = ""
                self.last_mode_sent = None
                self.command_event.set()
                return
            except (SerialException, OSError) as exc:
                errors.append(f"{port_name}: {exc}")

        self.serial_port = None
        self.actual_port = ""
        self.last_serial_error = " | ".join(errors)[-500:]

    def _close_serial(self) -> None:
        with self.serial_lock:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except (SerialException, OSError):
                    pass
            self.serial_port = None
            self.actual_port = ""

    def reconnect(self) -> None:
        self._close_serial()
        self.last_serial_attempt = 0.0
        self.last_serial_error = "正在重新连接"
        self.command_event.set()

    def _write_line(self, line: str) -> bool:
        self._connect_serial_if_needed()
        port = self.serial_port
        if port is None:
            return False
        try:
            with self.serial_lock:
                port.write((line.rstrip("\n") + "\n").encode("ascii"))
                port.flush()
            return True
        except (SerialException, OSError) as exc:
            self.last_serial_error = str(exc)
            self._close_serial()
            return False

    def _send_motors(self, motors: tuple[int, int, int, int]) -> None:
        values = tuple(clamp_motor(value) for value in motors)
        if self._write_line(f"M,{values[0]},{values[1]},{values[2]},{values[3]}"):
            self.last_sent = values

    def _send_speed(self, left: float, right: float) -> None:
        if self._write_line(f"V,{left:.1f},{right:.1f}"):
            self.last_speed_target = (float(left), float(right))

    def _send_pid(self) -> None:
        with self.lock:
            gains = (self.config.kp, self.config.ki, self.config.kd)
        self._write_line(f"KP,{gains[0]:.3f}")
        self._write_line(f"KI,{gains[1]:.3f}")
        self._write_line(f"KD,{gains[2]:.3f}")

    def update_keys(self, payload: dict[str, Any]) -> None:
        valid = {"w", "a", "s", "d", "q", "e", "z", "c"}
        keys_value = payload.get("keys", [])
        if isinstance(keys_value, list):
            keys = {str(key).lower() for key in keys_value if str(key).lower() in valid}
        else:
            # Compatibility with the older {w:true, a:false, ...} endpoint.
            keys = {key for key in valid if bool(payload.get(key, False))}

        with self.lock:
            self.held_keys = keys
            self.last_client_update = time.monotonic()
            self.client_connected = True
        self.command_event.set()

    def emergency_stop(self) -> None:
        with self.lock:
            self.held_keys.clear()
            self.client_connected = False
            self.current_action = "STOP"
        self.command_event.set()
        # Best effort immediate stop; the control loop repeats it.
        self._send_speed(0.0, 0.0)
        self._send_motors((0, 0, 0, 0))

    def update_config(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            old_mode = self.config.speed_mode
            for field in (
                "straight_pwm",
                "pivot_pwm",
                "curve_outer_pwm",
                "curve_inner_pwm",
            ):
                if field in payload:
                    setattr(self.config, field, clamp_pwm(payload[field]))

            if "speed_mode" in payload:
                self.config.speed_mode = bool(payload["speed_mode"])
            if "target_speed" in payload:
                self.config.target_speed = float(payload["target_speed"])
            for field in ("kp", "ki", "kd"):
                if field in payload:
                    setattr(self.config, field, float(payload[field]))

            self.config.normalise()
            self._save_config()
            new_mode = self.config.speed_mode
            result = asdict(self.config)

        if old_mode and not new_mode:
            self._send_speed(0.0, 0.0)
        if any(name in payload for name in ("kp", "ki", "kd")):
            self._send_pid()
        self.command_event.set()
        return result

    def _resolve_action(self) -> str:
        with self.lock:
            keys = set(self.held_keys)

        # Dedicated diagonal keys have priority.
        for key, action in (("q", "FL"), ("e", "FR"), ("z", "BL"), ("c", "BR")):
            if key in keys:
                return action

        forward = int("w" in keys) - int("s" in keys)
        turn = int("d" in keys) - int("a" in keys)
        if forward == 0 and turn == 0:
            return "STOP"
        if forward > 0:
            return "FL" if turn < 0 else "FR" if turn > 0 else "F"
        if forward < 0:
            return "BL" if turn < 0 else "BR" if turn > 0 else "B"
        return "PL" if turn < 0 else "PR"

    @staticmethod
    def _raw_outputs(action: str, cfg: RobotConfig) -> tuple[int, int, int, int]:
        s = cfg.straight_pwm
        p = cfg.pivot_pwm
        outer = cfg.curve_outer_pwm
        inner = cfg.curve_inner_pwm
        return {
            "F": (s, s, s, s),
            "B": (-s, -s, -s, -s),
            "PL": (p, -p, -p, p),
            "PR": (-p, p, p, -p),
            "FL": (outer, inner, inner, outer),
            "FR": (inner, outer, outer, inner),
            "BL": (-inner, -outer, -outer, -inner),
            "BR": (-outer, -inner, -inner, -outer),
            "STOP": (0, 0, 0, 0),
        }[action]

    @staticmethod
    def _speed_outputs(
        action: str,
        cfg: RobotConfig,
    ) -> tuple[float, float, tuple[int, int, int, int]]:
        t = cfg.target_speed
        half = t * 0.5
        # Keep the behavior of the previous combined controller.
        left, right, m1, m2 = {
            "F": (t, t, 0, 0),
            "B": (-t, -t, 0, 0),
            "PL": (-t, t, -cfg.pivot_pwm, cfg.pivot_pwm),
            "PR": (t, -t, cfg.pivot_pwm, -cfg.pivot_pwm),
            "FL": (half, t, cfg.curve_inner_pwm, cfg.curve_outer_pwm),
            "FR": (t, half, cfg.curve_outer_pwm, cfg.curve_inner_pwm),
            "BL": (-t, -half, -cfg.curve_inner_pwm, -cfg.curve_outer_pwm),
            "BR": (-half, -t, -cfg.curve_outer_pwm, -cfg.curve_inner_pwm),
            "STOP": (0.0, 0.0, 0, 0),
        }[action]
        return left, right, (m1, m2, 0, 0)

    def _send_current_command(self) -> None:
        action = self._resolve_action()
        with self.lock:
            cfg = RobotConfig(**asdict(self.config)).normalise()
            self.current_action = action

        if cfg.speed_mode:
            if self.last_mode_sent is not True:
                self._send_pid()
            left, right, motors = self._speed_outputs(action, cfg)
            self._send_speed(left, right)
            self._send_motors(motors)
            self.last_mode_sent = True
        else:
            if self.last_mode_sent is True:
                self._send_speed(0.0, 0.0)
            self._send_motors(self._raw_outputs(action, cfg))
            self.last_mode_sent = False

    def _query_telemetry(self) -> None:
        self._write_line("IMU")
        self._write_line("SPD")

    def _read_replies(self) -> None:
        port = self.serial_port
        if port is None:
            return
        try:
            while port.in_waiting:
                with self.serial_lock:
                    line = port.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("IMU,"):
                    parts = line.split(",")
                    if len(parts) == 4:
                        try:
                            self.imu = (float(parts[1]), float(parts[2]), float(parts[3]))
                        except ValueError:
                            pass
                    continue
                if line.startswith("SPD,"):
                    parts = line.split(",")
                    if len(parts) == 7:
                        try:
                            self.spd = {
                                "curL": float(parts[1]),
                                "curR": float(parts[2]),
                                "tgtL": float(parts[3]),
                                "tgtR": float(parts[4]),
                                "pidL": int(float(parts[5])),
                                "pidR": int(float(parts[6])),
                            }
                        except ValueError:
                            pass
                    continue
                self.last_serial_reply = line
        except (SerialException, OSError) as exc:
            self.last_serial_error = str(exc)
            self._close_serial()

    def _control_loop(self) -> None:
        next_send = 0.0
        next_query = 0.0
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                if (
                    self.client_connected
                    and now - self.last_client_update > CLIENT_TIMEOUT_SECONDS
                ):
                    self.held_keys.clear()
                    self.client_connected = False
                    self.current_action = "STOP"
                    self.command_event.set()

            if now >= next_send or self.command_event.is_set():
                self.command_event.clear()
                self._send_current_command()
                next_send = now + SEND_INTERVAL_SECONDS

            if now >= next_query:
                self._query_telemetry()
                next_query = now + QUERY_INTERVAL_SECONDS

            self._read_replies()
            self.command_event.wait(timeout=0.01)

        for _ in range(4):
            self._send_speed(0.0, 0.0)
            self._send_motors((0, 0, 0, 0))
            time.sleep(0.05)
        self._close_serial()

    def status(self) -> dict[str, Any]:
        with self.lock:
            cfg = asdict(self.config)
            keys = sorted(self.held_keys)
            client_connected = self.client_connected
            action = self.current_action
        return {
            "serial": {
                "online": self.serial_connected,
                "requested_port": self.requested_port,
                "actual_port": self.actual_port,
                "reply": self.last_serial_reply,
                "error": self.last_serial_error,
            },
            "client_connected": client_connected,
            "keys": keys,
            "action": action,
            "action_name": ACTION_NAMES[action],
            "motors": {
                "M1_right_front": self.last_sent[0],
                "M2_left_front": self.last_sent[1],
                "M3_left_rear": self.last_sent[2],
                "M4_right_rear": self.last_sent[3],
            },
            "speed_target": {
                "left": self.last_speed_target[0],
                "right": self.last_speed_target[1],
            },
            "imu": None
            if self.imu is None
            else {"roll": self.imu[0], "pitch": self.imu[1], "yaw": self.imu[2]},
            "speed": self.spd,
            "config": cfg,
        }

    def shutdown(self) -> None:
        self.stop_event.set()
        self.command_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)


HTML_PAGE = r"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Loss Hunters Robot Control</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --bg: #0b0e13;
      --panel: #141923;
      --panel2: #1b2230;
      --line: #2a3445;
      --text: #f3f6fb;
      --muted: #95a1b3;
      --blue: #3b82f6;
      --green: #22c55e;
      --red: #ef4444;
      --amber: #f59e0b;
    }
    * { box-sizing: border-box; }
    body { margin: 0; background: var(--bg); color: var(--text); min-height: 100vh; }
    header { padding: 18px clamp(16px, 3vw, 34px); border-bottom: 1px solid var(--line); display:flex; justify-content:space-between; align-items:center; gap:16px; position:sticky; top:0; background:rgba(11,14,19,.94); backdrop-filter: blur(12px); z-index:10; }
    h1 { margin: 0; font-size: clamp(19px, 2.4vw, 28px); }
    .subtitle { color: var(--muted); font-size: 13px; margin-top: 4px; }
    .layout { padding: 18px clamp(14px, 3vw, 34px) 34px; display:grid; grid-template-columns:minmax(0, 1.55fr) minmax(310px, .9fr); gap:18px; max-width:1500px; margin:auto; }
    .card { background:var(--panel); border:1px solid var(--line); border-radius:18px; padding:16px; box-shadow:0 10px 30px rgba(0,0,0,.18); }
    .card h2 { font-size:16px; margin:0 0 14px; }
    .video-wrap { aspect-ratio:4/3; border-radius:14px; overflow:hidden; background:#05070a; display:flex; align-items:center; justify-content:center; }
    .video-wrap img { width:100%; height:100%; object-fit:contain; }
    .status-row { display:grid; grid-template-columns:repeat(4, minmax(0, 1fr)); gap:10px; margin-top:12px; }
    .status-pill { background:var(--panel2); border:1px solid var(--line); border-radius:12px; padding:10px; min-width:0; }
    .status-pill span { display:block; color:var(--muted); font-size:11px; margin-bottom:3px; }
    .status-pill strong { display:block; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; font-size:13px; }
    .dot { display:inline-block; width:9px; height:9px; border-radius:50%; background:var(--red); margin-right:7px; }
    .dot.online { background:var(--green); box-shadow:0 0 10px rgba(34,197,94,.75); }
    .controls { display:grid; grid-template-columns:repeat(3, minmax(74px, 100px)); gap:10px; justify-content:center; }
    button { border:1px solid var(--line); border-radius:13px; background:var(--panel2); color:var(--text); cursor:pointer; font:inherit; transition:.12s ease; }
    button:hover { border-color:#52627a; transform:translateY(-1px); }
    button:active, button.active { background:var(--blue); border-color:#72a7ff; transform:translateY(0); }
    .move { min-height:70px; font-size:20px; font-weight:800; touch-action:none; user-select:none; }
    .move small { display:block; font-size:11px; color:#cbd5e1; font-weight:500; margin-top:3px; }
    .stop { background:#5d1620; border-color:#8d2635; }
    .stop:hover, .stop:active { background:var(--red); }
    .toolbar { display:flex; flex-wrap:wrap; gap:9px; margin-top:14px; }
    .toolbar button { padding:10px 13px; }
    .primary { background:#164a9b; border-color:#2668c9; }
    .danger { background:#5d1620; border-color:#8d2635; }
    .settings-grid { display:grid; grid-template-columns:1fr 92px; gap:10px 12px; align-items:center; }
    label { color:#d8dee9; font-size:13px; }
    input[type=number] { width:100%; background:#0d1118; border:1px solid var(--line); color:var(--text); border-radius:10px; padding:9px; }
    .mode-line { display:flex; align-items:center; justify-content:space-between; gap:12px; padding:11px 0 14px; border-bottom:1px solid var(--line); margin-bottom:14px; }
    .switch { position:relative; width:48px; height:27px; }
    .switch input { display:none; }
    .slider { position:absolute; inset:0; background:#303a4a; border-radius:99px; cursor:pointer; }
    .slider:before { content:""; position:absolute; width:21px; height:21px; left:3px; top:3px; background:white; border-radius:50%; transition:.18s; }
    .switch input:checked + .slider { background:var(--blue); }
    .switch input:checked + .slider:before { transform:translateX(21px); }
    .telemetry { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
    .metric { background:var(--panel2); border:1px solid var(--line); border-radius:12px; padding:10px; }
    .metric span { color:var(--muted); font-size:11px; }
    .metric strong { display:block; font-size:15px; margin-top:3px; }
    .log { white-space:pre-wrap; word-break:break-word; background:#090c11; border:1px solid var(--line); border-radius:12px; padding:11px; min-height:70px; color:#cbd5e1; font:12px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace; }
    .hint { color:var(--muted); font-size:12px; line-height:1.55; margin-top:12px; }
    .toast { position:fixed; right:18px; bottom:18px; background:#111827; border:1px solid var(--line); border-radius:12px; padding:11px 14px; opacity:0; transform:translateY(10px); pointer-events:none; transition:.2s; z-index:30; }
    .toast.show { opacity:1; transform:none; }
    @media (max-width: 900px) { .layout { grid-template-columns:1fr; } .status-row { grid-template-columns:1fr 1fr; } }
    @media (max-width: 480px) { header { position:static; } .layout { padding:10px; } .card { border-radius:14px; padding:12px; } .move { min-height:64px; } .telemetry { grid-template-columns:1fr; } }
  </style>
</head>
<body>
<header>
  <div>
    <h1>Loss Hunters · Robot Control</h1>
    <div class="subtitle">视频、遥控、参数和遥测都在同一网页</div>
  </div>
  <button class="danger" id="headerStop" style="padding:11px 16px;font-weight:800">紧急停止</button>
</header>

<main class="layout">
  <section>
    <div class="card">
      <h2>实时视频</h2>
      <div class="video-wrap"><img src="/video_feed" alt="摄像头视频流"></div>
      <div class="status-row">
        <div class="status-pill"><span>摄像头</span><strong id="cameraState"><i class="dot"></i>读取中</strong></div>
        <div class="status-pill"><span>Arduino</span><strong id="serialState"><i class="dot"></i>连接中</strong></div>
        <div class="status-pill"><span>当前动作</span><strong id="actionState">停止</strong></div>
        <div class="status-pill"><span>控制连接</span><strong id="clientState">等待网页心跳</strong></div>
      </div>
      <div class="toolbar">
        <button class="primary" id="captureBtn">拍摄当前画面</button>
        <button id="reconnectBtn">重新连接 Arduino</button>
        <button class="danger" id="videoStop">紧急停止</button>
      </div>
    </div>

    <div class="card">
      <h2>网页遥控</h2>
      <div class="controls">
        <button class="move" data-key="q">Q<small>前进左转</small></button>
        <button class="move" data-key="w">W / ↑<small>前进</small></button>
        <button class="move" data-key="e">E<small>前进右转</small></button>
        <button class="move" data-key="a">A / ←<small>原地左转</small></button>
        <button class="move stop" id="padStop">STOP<small>空格</small></button>
        <button class="move" data-key="d">D / →<small>原地右转</small></button>
        <button class="move" data-key="z">Z<small>后退左转</small></button>
        <button class="move" data-key="s">S / ↓<small>后退</small></button>
        <button class="move" data-key="c">C<small>后退右转</small></button>
      </div>
      <div class="hint">按住按钮或键盘才运动，松开立即停车。页面失焦、网络断开或超过 0.8 秒没有网页心跳也会自动停车。</div>
    </div>

    <div class="card">
      <h2>遥测</h2>
      <div class="telemetry">
        <div class="metric"><span>IMU Roll</span><strong id="roll">—</strong></div>
        <div class="metric"><span>IMU Pitch</span><strong id="pitch">—</strong></div>
        <div class="metric"><span>IMU Yaw</span><strong id="yaw">—</strong></div>
        <div class="metric"><span>轮速 L / R</span><strong id="wheelSpeed">—</strong></div>
        <div class="metric"><span>目标速度 L / R</span><strong id="targetWheelSpeed">—</strong></div>
        <div class="metric"><span>电机输出 M1…M4</span><strong id="motorOutput">0 / 0 / 0 / 0</strong></div>
      </div>
    </div>
  </section>

  <aside>
    <div class="card">
      <h2>控制模式与参数</h2>
      <div class="mode-line">
        <div><strong>速度 PID 模式</strong><div class="subtitle">关闭时使用直接 PWM</div></div>
        <label class="switch"><input type="checkbox" id="speed_mode"><span class="slider"></span></label>
      </div>
      <div class="settings-grid">
        <label for="straight_pwm">直行 PWM</label><input id="straight_pwm" type="number" min="0" max="255">
        <label for="pivot_pwm">原地转向 PWM</label><input id="pivot_pwm" type="number" min="0" max="255">
        <label for="curve_outer_pwm">转弯外侧 PWM</label><input id="curve_outer_pwm" type="number" min="0" max="255">
        <label for="curve_inner_pwm">转弯内侧 PWM</label><input id="curve_inner_pwm" type="number" min="0" max="255">
        <label for="target_speed">目标速度 pps</label><input id="target_speed" type="number" min="0" max="200" step="5">
        <label for="kp">Kp</label><input id="kp" type="number" min="0" step="0.1">
        <label for="ki">Ki</label><input id="ki" type="number" min="0" step="0.1">
        <label for="kd">Kd</label><input id="kd" type="number" min="0" step="0.01">
      </div>
      <div class="toolbar">
        <button class="primary" id="applyBtn">应用并保存</button>
        <button id="speedMinus">速度 −5</button>
        <button id="speedPlus">速度 +5</button>
      </div>
    </div>

    <div class="card">
      <h2>系统状态</h2>
      <div class="log" id="systemLog">正在读取……</div>
      <div class="hint">默认地址：<strong>http://100.80.46.54:5000/</strong><br>正常操作不再需要 VS Code 终端。</div>
    </div>
  </aside>
</main>
<div class="toast" id="toast"></div>

<script>
const movementKeys = new Set(["w","a","s","d","q","e","z","c"]);
const pressed = new Set();
const buttons = [...document.querySelectorAll("[data-key]")];
let configLoaded = false;
let sendBusy = false;

function isEditing(event) {
  const tag = event.target?.tagName?.toLowerCase();
  return tag === "input" || tag === "textarea" || tag === "select";
}
function normalizedKey(event) {
  const arrows = {ArrowUp:"w", ArrowDown:"s", ArrowLeft:"a", ArrowRight:"d"};
  return arrows[event.key] || event.key.toLowerCase();
}
function renderPressed() {
  for (const button of buttons) button.classList.toggle("active", pressed.has(button.dataset.key));
}
async function sendKeys() {
  if (sendBusy) return;
  sendBusy = true;
  try {
    await fetch("/api/keys", {
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({keys:[...pressed]}),
      keepalive:true,
    });
  } catch (_) {} finally { sendBusy = false; }
}
function setPressed(key, value) {
  if (!movementKeys.has(key)) return;
  if (value) pressed.add(key); else pressed.delete(key);
  renderPressed();
  sendKeys();
}
async function emergencyStop(show=true) {
  pressed.clear();
  renderPressed();
  try { await fetch("/api/stop", {method:"POST", keepalive:true}); } catch (_) {}
  if (show) toast("已发送紧急停止");
}

document.addEventListener("keydown", event => {
  if (isEditing(event)) return;
  if (event.code === "Space") {
    event.preventDefault();
    emergencyStop();
    return;
  }
  const key = normalizedKey(event);
  if (movementKeys.has(key)) {
    event.preventDefault();
    if (!event.repeat) setPressed(key, true);
  }
});
document.addEventListener("keyup", event => {
  if (isEditing(event)) return;
  const key = normalizedKey(event);
  if (movementKeys.has(key)) {
    event.preventDefault();
    setPressed(key, false);
  }
});
for (const button of buttons) {
  const key = button.dataset.key;
  button.addEventListener("pointerdown", event => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    setPressed(key, true);
  });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) {
    button.addEventListener(name, event => {
      event.preventDefault();
      setPressed(key, false);
    });
  }
}
for (const id of ["headerStop", "videoStop", "padStop"]) {
  document.getElementById(id).addEventListener("click", () => emergencyStop());
}
window.addEventListener("blur", () => emergencyStop(false));
document.addEventListener("visibilitychange", () => { if (document.hidden) emergencyStop(false); });
window.addEventListener("beforeunload", () => {
  navigator.sendBeacon("/api/stop", new Blob([], {type:"application/json"}));
});
setInterval(sendKeys, 180);

function toast(message) {
  const box = document.getElementById("toast");
  box.textContent = message;
  box.classList.add("show");
  clearTimeout(window.toastTimer);
  window.toastTimer = setTimeout(() => box.classList.remove("show"), 1800);
}
function dot(online, text) { return `<i class="dot ${online ? "online" : ""}"></i>${text}`; }
function value(id) { return document.getElementById(id).value; }

async function applyConfig(extra={}) {
  const payload = {
    straight_pwm:Number(value("straight_pwm")),
    pivot_pwm:Number(value("pivot_pwm")),
    curve_outer_pwm:Number(value("curve_outer_pwm")),
    curve_inner_pwm:Number(value("curve_inner_pwm")),
    speed_mode:document.getElementById("speed_mode").checked,
    target_speed:Number(value("target_speed")),
    kp:Number(value("kp")), ki:Number(value("ki")), kd:Number(value("kd")),
    ...extra,
  };
  try {
    const response = await fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "设置失败");
    fillConfig(result.config);
    toast("参数已应用并保存");
  } catch (error) { toast(error.message); }
}
function fillConfig(config) {
  for (const name of ["straight_pwm","pivot_pwm","curve_outer_pwm","curve_inner_pwm","target_speed","kp","ki","kd"]) {
    if (document.activeElement?.id !== name) document.getElementById(name).value = config[name];
  }
  if (document.activeElement?.id !== "speed_mode") document.getElementById("speed_mode").checked = config.speed_mode;
  configLoaded = true;
}
document.getElementById("applyBtn").addEventListener("click", () => applyConfig());
document.getElementById("speed_mode").addEventListener("change", () => applyConfig());
document.getElementById("speedMinus").addEventListener("click", () => {
  document.getElementById("target_speed").value = Math.max(0, Number(value("target_speed")) - 5); applyConfig();
});
document.getElementById("speedPlus").addEventListener("click", () => {
  document.getElementById("target_speed").value = Math.min(200, Number(value("target_speed")) + 5); applyConfig();
});
document.getElementById("reconnectBtn").addEventListener("click", async () => {
  await fetch("/api/reconnect", {method:"POST"}); toast("正在重新连接 Arduino");
});
document.getElementById("captureBtn").addEventListener("click", async () => {
  try {
    const response = await fetch("/api/capture", {method:"POST"});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "拍摄失败");
    window.open(result.url, "_blank");
    toast("照片已保存");
  } catch (error) { toast(error.message); }
});

async function refreshStatus() {
  try {
    const response = await fetch("/api/status", {cache:"no-store"});
    const data = await response.json();
    const serial = data.robot.serial;
    const camera = data.camera;
    document.getElementById("cameraState").innerHTML = dot(camera.online, camera.status);
    document.getElementById("serialState").innerHTML = dot(serial.online, serial.online ? (serial.actual_port || "Online") : "Offline");
    document.getElementById("actionState").textContent = `${data.robot.action_name} (${data.robot.action})`;
    document.getElementById("clientState").textContent = data.robot.client_connected ? "网页控制在线" : "安全停车";

    const imu = data.robot.imu;
    document.getElementById("roll").textContent = imu ? `${imu.roll.toFixed(2)}°` : "—";
    document.getElementById("pitch").textContent = imu ? `${imu.pitch.toFixed(2)}°` : "—";
    document.getElementById("yaw").textContent = imu ? `${imu.yaw.toFixed(2)}°` : "—";
    const speed = data.robot.speed;
    document.getElementById("wheelSpeed").textContent = speed ? `${speed.curL.toFixed(1)} / ${speed.curR.toFixed(1)} pps` : "—";
    document.getElementById("targetWheelSpeed").textContent = speed ? `${speed.tgtL.toFixed(1)} / ${speed.tgtR.toFixed(1)} pps` : "—";
    const m = data.robot.motors;
    document.getElementById("motorOutput").textContent = `${m.M1_right_front} / ${m.M2_left_front} / ${m.M3_left_rear} / ${m.M4_right_rear}`;
    if (!configLoaded) fillConfig(data.robot.config);

    const lines = [
      `Web: ${data.web_url}`,
      `Serial request: ${serial.requested_port}`,
      `Serial actual: ${serial.actual_port || "—"}`,
      `Arduino reply: ${serial.reply || "—"}`,
      `Serial error: ${serial.error || "—"}`,
      `Camera error: ${camera.error || "—"}`,
      `Held keys: ${data.robot.keys.join("+") || "none"}`,
    ];
    document.getElementById("systemLog").textContent = lines.join("\n");
  } catch (error) {
    document.getElementById("systemLog").textContent = `网页后端连接失败：${error}`;
  }
}
refreshStatus();
setInterval(refreshStatus, 500);
</script>
</body>
</html>
"""


def create_app(
    controller: RobotController,
    camera: CameraStreamer,
    captures_dir: Path,
    display_url: str,
) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index() -> str:
        return HTML_PAGE

    @app.get("/video_feed")
    def video_feed():
        if not camera.online:
            return jsonify({"ok": False, "error": camera.error or camera.status}), 503
        return Response(
            camera.iter_mjpeg(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

    @app.post("/api/keys")
    def api_keys():
        payload = request.get_json(silent=True) or {}
        controller.update_keys(payload)
        return jsonify({"ok": True})

    @app.post("/api/stop")
    def api_stop():
        controller.emergency_stop()
        return jsonify({"ok": True})

    @app.post("/api/reconnect")
    def api_reconnect():
        controller.emergency_stop()
        controller.reconnect()
        return jsonify({"ok": True})

    @app.post("/api/config")
    def api_config():
        payload = request.get_json(silent=True) or {}
        try:
            config = controller.update_config(payload)
            return jsonify({"ok": True, "config": config})
        except (TypeError, ValueError) as exc:
            return jsonify({"ok": False, "error": f"参数格式错误：{exc}"}), 400

    @app.get("/api/status")
    def api_status():
        return jsonify(
            {
                "ok": True,
                "web_url": display_url,
                "robot": controller.status(),
                "camera": camera.status_dict(),
            }
        )

    @app.post("/api/capture")
    def api_capture():
        try:
            path = camera.save_snapshot(captures_dir)
            return jsonify({"ok": True, "filename": path.name, "url": f"/captures/{path.name}"})
        except (RuntimeError, OSError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 503

    @app.get("/captures/<path:filename>")
    def captures(filename: str):
        return send_from_directory(captures_dir, filename)

    @app.get("/health")
    def health():
        return jsonify(
            {
                "ok": True,
                "camera": camera.online,
                "serial": controller.serial_connected,
            }
        )

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="树莓派小车一体化网页控制器")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Arduino 串口；也可使用 auto")
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=5000)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--no-camera", action="store_true")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("robot_config.json"),
    )
    parser.add_argument(
        "--captures-dir",
        type=Path,
        default=Path(__file__).with_name("captures"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.captures_dir.mkdir(parents=True, exist_ok=True)

    camera = CameraStreamer(
        args.camera_width,
        args.camera_height,
        args.jpeg_quality,
        args.camera_fps,
    )
    if args.no_camera:
        camera.status = "已禁用"
    else:
        camera.start()

    controller = RobotController(args.port, args.config)
    controller.start()

    display_url = f"http://{get_display_ip()}:{args.web_port}/"
    app = create_app(controller, camera, args.captures_dir, display_url)

    shutdown_lock = threading.Lock()
    shut_down = False

    def cleanup(*_: Any) -> None:
        nonlocal shut_down
        with shutdown_lock:
            if shut_down:
                return
            shut_down = True
        controller.emergency_stop()
        controller.shutdown()
        camera.stop()

    atexit.register(cleanup)
    for signal_name in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(signal_name, lambda *_args: (_ for _ in ()).throw(KeyboardInterrupt()))
        except (ValueError, OSError):
            pass

    print("=" * 62)
    print("树莓派小车网页控制器已启动")
    print(f"网页地址：{display_url}")
    print(f"Arduino 串口请求：{args.port}")
    print("关闭程序时会自动向 Arduino 重复发送停车命令。")
    print("=" * 62)

    try:
        app.run(
            host=args.web_host,
            port=args.web_port,
            threaded=True,
            use_reloader=False,
            debug=False,
        )
    except KeyboardInterrupt:
        print("\n正在安全停车并退出……")
    finally:
        cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
