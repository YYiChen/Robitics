"""Terminal-compatible action state machine and Arduino serial bridge."""
from __future__ import annotations

import threading
import time
from dataclasses import asdict
import json
import os
from pathlib import Path

import serial

from control.card_control import CARD_COMMAND_ACK_TIMEOUT_SECONDS, CardControlMixin
from control.drive_config import (
    Config,
    default_profiles,
    legacy_scalar_profiles,
    normalize_profiles,
)
from control.motor_commands import raw_motor_output, speed_targets
from control.protocol import parse_protocol_line
from control.servo_control import ServoControlMixin

HEARTBEAT_SECONDS = 0.20
DIRECT_DRIVE_SECONDS = 0.05
AUTONOMOUS_DRIVE_LEASE_SECONDS = 0.35
SERVO_TICK_SECONDS = 0.05
CLIENT_TIMEOUT_SECONDS = 0.80
ACTIONS = {"STOP", "F", "SF", "B", "PL", "PR", "SPL", "SPR", "FL", "FR", "BL", "BR"}
# Q/E are steering-servo controls rather than motor profiles.
KEY_ACTIONS = {"w": "F", "slow": "SF", "a": "PL", "d": "PR", "s": "B", "x": "SPL", "c": "SPR"}
class RobotController(CardControlMixin, ServoControlMixin):
    heartbeat_seconds = HEARTBEAT_SECONDS
    client_timeout_seconds = CLIENT_TIMEOUT_SECONDS
    # Compatibility names retained for existing diagnostics.
    _raw = staticmethod(raw_motor_output)
    _speed = staticmethod(speed_targets)
    def __init__(
        self,
        port: str,
        config_path: Path | None = None,
        legacy_config_path: Path | None = None,
        config_store=None,
    ) -> None:
        default_path = Path(__file__).with_name("drive_config.json")
        self.port, self.config_path, self.lock = port, Path(config_path or default_path).expanduser(), threading.RLock()
        # Explicit paths remain the compatibility/testing contract.  The
        # assembled formal service passes the unified store instead.
        self.config_store = config_store if config_path is None else None
        self.legacy_config_path = (
            Path(legacy_config_path).expanduser()
            if legacy_config_path is not None
            else Path(__file__).with_name("robot_config.json") if config_path is None else None
        )
        self.config_io_lock = threading.Lock()
        self.config, migrated_legacy, self.config_source, self.config_error = self._load_config()
        # Existing users already have their tuned values in robot_config.json.
        # Copy them once to the dedicated drive file; never overwrite the old
        # file, and never let ordinary source edits replace the tuned values.
        if migrated_legacy:
            self._save_config()
        self.serial_lock = threading.RLock()
        self.serial = None; self.action = "STOP"; self.held_keys: set[str] = set(); self.last_client_seen = 0.0
        self.direct_drive: tuple[int, int] | None = None
        self.direct_drive_owner: str | None = None
        self.direct_drive_expires_at = 0.0
        self.steering_direction = 0; self.last_steering_seen = 0.0
        self._last_sent_steering_direction: int | None = None
        self._last_steering_sent_at = 0.0
        self._last_sent_servo_limits: tuple[float, float] | None = None
        self._servo_target_angle: float | None = None
        self.imu = self.speed = self.ultrasonic = None; self.servo_angle: int | None = None; self.motor_output: list[int] | None = None; self.card_feed_state = self.card_deal_state = "idle"; self.card_motor_protocol = "unknown"; self.card_command_reply = ""; self._card_reply_sequence = 0; self._card_reply_waiting_for = None
        self._card_reply_condition = threading.Condition(self.lock)
        self._card_command_lock = threading.Lock()
        self._deal_request_lock = threading.Lock()
        self._last_deal_request_token = ""
        self._last_deal_request_result: dict | None = None
        self._card_event_listeners: list = []
        self.reply = self.error = ""; self.last_rx = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._shutdown_lock = threading.Lock()
        self._stopped = False

    @staticmethod
    def _read_config(path: Path) -> tuple[Config | None, str]:
        try:
            data = json.loads(path.read_text(encoding="utf-8")); config = Config()
            for key in ("speed_mode", "target_speed", "kp", "ki", "kd", "straight_pwm", "pivot_pwm", "curve_outer_pwm", "curve_inner_pwm", "servo_center_angle", "servo_speed_dps", "servo_acceleration_dps2", "servo_qe_reversed"):
                if key in data: setattr(config, key, type(getattr(config, key))(data[key]))
            raw_profiles = data.get("profiles")
            config.profiles = normalize_profiles(raw_profiles) if isinstance(raw_profiles, dict) else legacy_scalar_profiles(config)
            return config, ""
        except FileNotFoundError:
            return None, ""
        except (json.JSONDecodeError, TypeError, ValueError, OSError) as exc:
            return None, f"无法读取配置文件 {path}: {exc}"

    def _load_config(self) -> tuple[Config, bool, str, str]:
        if self.config_store is not None:
            try:
                data = self.config_store.read_section("drive")
                config = Config()
                for key in ("speed_mode", "target_speed", "kp", "ki", "kd", "straight_pwm", "pivot_pwm", "curve_outer_pwm", "curve_inner_pwm", "servo_center_angle", "servo_speed_dps", "servo_acceleration_dps2", "servo_qe_reversed"):
                    if key in data:
                        setattr(config, key, type(getattr(config, key))(data[key]))
                raw_profiles = data.get("profiles")
                config.profiles = normalize_profiles(raw_profiles) if isinstance(raw_profiles, dict) else legacy_scalar_profiles(config)
                return config, False, "unified_config:drive", ""
            except (TypeError, ValueError, OSError) as exc:
                return Config(), False, "code_defaults", f"无法读取统一 drive 配置: {exc}"
        config, error = self._read_config(self.config_path)
        if config is not None:
            return config, False, "drive_config", ""
        if self.legacy_config_path is not None:
            legacy, legacy_error = self._read_config(self.legacy_config_path)
            if legacy is not None:
                return legacy, True, "legacy_robot_config", ""
            error = error or legacy_error
        return Config(), False, "code_defaults", error

    def _save_config(self, snapshot: dict | None = None) -> None:
        data = snapshot if snapshot is not None else asdict(self.config)
        if self.config_store is not None:
            self.config_store.write_section("drive", data)
            self.config_source, self.config_error = "unified_config:drive", ""
            return
        payload = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        # Serialize writes and flush the file before replacing the live copy.
        # This prevents a Ctrl+C or concurrent request from leaving a partial
        # JSON file that makes the next process fall back to defaults.
        with self.config_io_lock:
            temporary = self.config_path.with_name(f".{self.config_path.name}.{os.getpid()}.tmp")
            with temporary.open("w", encoding="utf-8", newline="\n") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            temporary.replace(self.config_path)
        self.config_source, self.config_error = "drive_config", ""

    def start(self) -> None:
        with self._shutdown_lock:
            if self._thread and self._thread.is_alive(): return
            self._stopped = False
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True, name="robot-control")
            self._thread.start()

    def stop(self) -> None:
        """Stop the control loop while preserving the latest configuration."""
        with self._shutdown_lock:
            if self._stopped: return
            self._stopped = True
            thread = self._thread
        self._stop.set()
        with self.lock:
            self.held_keys.clear()
            self.direct_drive = None
            self.direct_drive_owner = None
            self.direct_drive_expires_at = 0.0
            self.action = "STOP"; self.steering_direction = 0
            snapshot = asdict(self.config)
        try:
            self._save_config(snapshot)
        except Exception as exc:
            self.error = f"配置保存失败: {exc}"
        if thread and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._close_serial(send_stop=True)
    def _connect(self) -> None:
        with self.serial_lock:
            if self.serial and self.serial.is_open: return
            try: self.serial = serial.Serial(self.port, 9600, timeout=.02, write_timeout=.5); time.sleep(2.5); self.error = ""
            except Exception as exc: self.serial, self.error = None, str(exc)
    def _close_serial(self, send_stop: bool = False) -> None:
        with self.serial_lock:
            port, self.serial = self.serial, None
            self._last_sent_steering_direction = None
            self._last_sent_servo_limits = None
            self._last_steering_sent_at = 0.0
            if port:
                try:
                    if send_stop and port.is_open: port.write(b"STOP\n"); port.flush()
                    if port.is_open: port.close()
                except Exception: pass
    def _write(self, command: str) -> bool:
        self._connect()
        with self.serial_lock:
            if self.serial:
                try:
                    self.serial.write((command + "\n").encode("ascii")); self.serial.flush()
                    return True
                except Exception as exc:
                    self.error = str(exc); self._close_serial()
        return False
    def select_action(self, action: str) -> str:
        action = action.upper()
        if action not in ACTIONS: raise ValueError("未知动作")
        with self.lock:
            # Compatibility endpoint for terminal/curl diagnostics.  The web
            # interface itself uses update_keys so releasing a key stops now.
            self.held_keys.clear(); self.direct_drive = None; self.direct_drive_owner = None; self.direct_drive_expires_at = 0.0
            self.action, self.last_client_seen = action, time.monotonic()
        return action

    def set_direct_drive(
        self,
        right_pwm: object,
        left_pwm: object,
        *,
        owner: str = "autonomous",
        lease_seconds: float = AUTONOMOUS_DRIVE_LEASE_SECONDS,
    ) -> tuple[int, int]:
        """Apply bounded M1/M2 PWM without changing persisted profiles.

        This is intended for short-lived closed-loop clients such as visual
        line following.  The normal 0.8 second client heartbeat timeout still
        converts it to STOP when the client disappears.
        """
        try:
            right, left = int(right_pwm), int(left_pwm)
        except (TypeError, ValueError):
            raise ValueError("左右电机 PWM 必须是整数") from None
        if not -255 <= right <= 255 or not -255 <= left <= 255:
            raise ValueError("左右电机 PWM 必须在 -255 到 255 之间")
        with self.lock:
            now = time.monotonic()
            self.held_keys.clear()
            self.direct_drive = (right, left)
            self.direct_drive_owner = str(owner)
            self.direct_drive_expires_at = now + max(0.0, float(lease_seconds)) if self.direct_drive_owner == "autonomous" else 0.0
            self.action, self.last_client_seen = "PID", now
        return right, left
    @staticmethod
    def _action_from_keys(keys: set[str]) -> str:
        if "x" in keys: return "SPL"
        if "c" in keys: return "SPR"
        forward, turn = int("w" in keys) - int("s" in keys), int("d" in keys) - int("a" in keys)
        if forward > 0: return "FL" if turn < 0 else "FR" if turn > 0 else "F"
        if forward < 0: return "BL" if turn < 0 else "BR" if turn > 0 else "B"
        if turn < 0: return "PL"
        if turn > 0: return "PR"
        return "SF" if "slow" in keys else "STOP"
    def update_keys(self, payload: dict) -> str:
        received = payload.get("keys", [])
        keys = {str(key).lower() for key in received if str(key).lower() in KEY_ACTIONS} if isinstance(received, list) else set()
        try: steering = int(payload.get("steering", 0))
        except (TypeError, ValueError): steering = 0
        steering = max(-1, min(1, steering))
        stop_requested = bool(payload.get("stop", False))
        with self.lock:
            now = time.monotonic()
            if stop_requested:
                self.held_keys.clear()
                self.direct_drive = None
                self.direct_drive_owner = None
                self.direct_drive_expires_at = 0.0
                self.action, self.last_client_seen = "STOP", now
                self.steering_direction, self.last_steering_seen = 0, now
                action = self.action
            else:
                # An empty browser heartbeat keeps the manual dead-man connection
                # alive but must not erase a route tracker's M1/M2 command.  A real
                # WASD key press still explicitly takes ownership from autonomy.
                if keys or self.direct_drive_owner != "autonomous":
                    self.direct_drive = None
                    self.direct_drive_owner = None
                    self.direct_drive_expires_at = 0.0
                self.held_keys, self.action, self.last_client_seen = keys, self._action_from_keys(keys), now
                if self.direct_drive is not None:
                    self.action = "PID"
                self.steering_direction, self.last_steering_seen = steering, now
                action = self.action
        if stop_requested:
            self._write("STOP")
        return action
    def heartbeat(self) -> None:
        with self.lock: self.last_client_seen = time.monotonic()
    def update_config(self, payload: dict) -> dict:
        with self.lock:
            for key, old in asdict(self.config).items():
                if key not in payload or key == "profiles": continue
                setattr(self.config, key, bool(payload[key]) if isinstance(old, bool) else type(old)(payload[key]))
            if "profiles" in payload: self.config.profiles = normalize_profiles(payload["profiles"])
            self.config.target_speed = max(0.0, min(200.0, self.config.target_speed))
            for key in ("straight_pwm", "pivot_pwm", "curve_outer_pwm", "curve_inner_pwm"):
                setattr(self.config, key, max(0, min(255, getattr(self.config, key))))
            self.config.servo_center_angle = max(0, min(180, self.config.servo_center_angle))
            self.config.servo_speed_dps = max(1.0, min(45.0, self.config.servo_speed_dps))
            self.config.servo_acceleration_dps2 = max(20.0, min(360.0, self.config.servo_acceleration_dps2))
            result = asdict(self.config)
        # SD-card I/O must not hold the control/heartbeat lock.
        self._save_config(result)
        return result
    def stop_now(self) -> None:
        with self.lock:
            self.held_keys.clear(); self.direct_drive = None; self.direct_drive_owner = None; self.direct_drive_expires_at = 0.0
            self.action = "STOP"; self.steering_direction = 0
        self._write("STOP")

    def reconnect(self) -> dict:
        """Drop the current USB serial session and open it again.

        Opening an Arduino serial port may reset the board, so the method also
        puts the controller in STOP and waits in _connect for boot to finish.
        """
        with self.lock:
            self.held_keys.clear(); self.direct_drive = None; self.direct_drive_owner = None; self.direct_drive_expires_at = 0.0
            self.action, self.last_client_seen = "STOP", 0.0; self.steering_direction = 0
            self.last_rx = 0.0; self.error = ""
        self._close_serial(send_stop=True)
        self._connect()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if self.last_rx: break
            time.sleep(.05)
        return self.status()
    def _parse(self, text: str) -> None:
        self.reply, self.last_rx = text, time.monotonic()
        with self._card_reply_condition:
            waiting_for = self._card_reply_waiting_for
            is_expected_card_reply = bool(
                waiting_for
                and (
                    text.startswith(f"OK:{waiting_for}")
                    or text == f"BUSY:{waiting_for}"
                    or text.startswith("ERR:")
                )
            )
            if is_expected_card_reply:
                self.card_command_reply = text
                self._card_reply_sequence += 1
                self._card_reply_condition.notify_all()
        for name, value in parse_protocol_line(text, self.motor_output).items():
            setattr(self, name, value)
        if text in {"FEED:DONE", "DEAL:DONE"}:
            with self.lock:
                listeners = tuple(self._card_event_listeners)
            for listener in listeners:
                try:
                    listener(text)
                except Exception:
                    # Telemetry/checkpoint listeners must never break serial RX.
                    pass

    def add_card_event_listener(self, listener) -> None:
        with self.lock:
            if listener not in self._card_event_listeners:
                self._card_event_listeners.append(listener)

    def remove_card_event_listener(self, listener) -> None:
        with self.lock:
            if listener in self._card_event_listeners:
                self._card_event_listeners.remove(listener)
    def _run(self) -> None:
        next_motor, next_query, next_servo_query = 0.0, 0.0, 0.0
        while not self._stop.wait(SERVO_TICK_SECONDS):
            now = time.monotonic()
            with self.lock:
                client_is_current = now - self.last_client_seen <= CLIENT_TIMEOUT_SECONDS
                autonomous_drive_is_current = (
                    self.direct_drive_owner == "autonomous"
                    and now <= self.direct_drive_expires_at
                )
                if self.direct_drive_owner == "autonomous" and not autonomous_drive_is_current:
                    self.direct_drive = None
                    self.direct_drive_owner = None
                    self.direct_drive_expires_at = 0.0
                    self.action = "STOP"
                action = self.action if client_is_current else "STOP"
                direct_drive = self.direct_drive if (client_is_current and autonomous_drive_is_current) else None
                if action == "STOP": self.held_keys.clear(); self.action = "STOP"
                cfg = Config(**asdict(self.config))
            self._sync_steering(now, cfg)
            if now >= next_servo_query:
                self._write("SVP")
                next_servo_query = now + HEARTBEAT_SECONDS
            if now < next_motor:
                continue
            if direct_drive is not None:
                # Visual P control owns the instantaneous M1/M2 command.
                # It is intentionally separate from Arduino wheel-speed PID.
                self._write("M," + ",".join(map(str, (*direct_drive, 0, 0))))
            elif cfg.speed_mode:
                left, right, front_left, front_right = speed_targets(action, cfg)
                self._write(f"V,{left:.1f},{right:.1f}")
                # The Arduino applies right/left PID to M1/M2 and ignores the
                # M3/M4 fields because those belong to the card mechanism.
                self._write("M,0,0,0,0")
            else:
                self._write("M," + ",".join(map(str, raw_motor_output(action, cfg))))
            next_motor = now + (DIRECT_DRIVE_SECONDS if direct_drive is not None else HEARTBEAT_SECONDS)
            with self.serial_lock:
                port = self.serial
                if port:
                    for _ in range(8):
                        line = port.readline().decode("ascii", "ignore").strip()
                        if line: self._parse(line)
            if now >= next_query:
                for command in ("IMU", "SPD", "US", "OUT"): self._write(command)
                next_query = now + .5
    def status(self) -> dict:
        with self.lock: cfg, action, seen, feed_state, deal_state, card_protocol, card_reply = asdict(self.config), self.action, self.last_client_seen, self.card_feed_state, self.card_deal_state, self.card_motor_protocol, self.card_command_reply
        serial_open = bool(self.serial and self.serial.is_open); age = time.monotonic() - self.last_rx if self.last_rx else None
        return {"serial":serial_open,"arduino_online":serial_open and age is not None and age <= 1.5,"last_rx_age":age,"error":self.error,"reply":self.reply,"config":cfg,"config_path":str(self.config_path),"config_source":self.config_source,"config_error":self.config_error,"action":action,"keys":sorted(self.held_keys),"client_online":time.monotonic()-seen<=CLIENT_TIMEOUT_SECONDS,"drive_owner":self.direct_drive_owner,"steering_direction":self.steering_direction,"imu":self.imu,"speed":self.speed,"ultrasonic":self.ultrasonic,"servo_angle":self.servo_angle,"motor_output":self.motor_output,"card_feed_state":feed_state,"card_deal_state":deal_state,"card_motor_protocol":card_protocol,"card_command_reply":card_reply}
