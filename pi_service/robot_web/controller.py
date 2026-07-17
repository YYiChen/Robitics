"""Terminal-compatible action state machine and Arduino serial bridge."""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path

import serial

HEARTBEAT_SECONDS = 0.20
SERVO_TICK_SECONDS = 0.05
CLIENT_TIMEOUT_SECONDS = 0.80
ACTIONS = {"STOP", "F", "SF", "B", "PL", "PR", "SPL", "SPR", "FL", "FR", "BL", "BR"}
# Q/E are steering-servo controls rather than motor profiles.
KEY_ACTIONS = {"w": "F", "r": "SF", "a": "PL", "d": "PR", "s": "B", "x": "SPL", "c": "SPR"}
PROFILE_ACTIONS = ("F", "SF", "B", "PL", "PR", "SPL", "SPR", "FL", "FR", "BL", "BR")
WHEELS = ("rf", "lf", "lr", "rr")

def default_profiles() -> dict[str, dict[str, int]]:
    return {
        "F": {"rf": 255, "lf": 255, "lr": 255, "rr": 255}, "SF": {"rf": 100, "lf": 100, "lr": 100, "rr": 100}, "B": {"rf": -255, "lf": -255, "lr": -255, "rr": -255},
        "PL": {"rf": 180, "lf": -180, "lr": -180, "rr": 180}, "PR": {"rf": -180, "lf": 180, "lr": 180, "rr": -180},
        "SPL": {"rf": 120, "lf": -120, "lr": -120, "rr": 120}, "SPR": {"rf": -120, "lf": 120, "lr": 120, "rr": -120},
        "FL": {"rf": 255, "lf": 60, "lr": 60, "rr": 255}, "FR": {"rf": 60, "lf": 255, "lr": 255, "rr": 60},
        "BL": {"rf": -255, "lf": -60, "lr": -60, "rr": -255}, "BR": {"rf": -60, "lf": -255, "lr": -255, "rr": -60},
    }

def normalize_profiles(raw: object) -> dict[str, dict[str, int]]:
    profiles = default_profiles()
    if not isinstance(raw, dict): return profiles
    for action in PROFILE_ACTIONS:
        values = raw.get(action)
        if not isinstance(values, dict): continue
        for wheel in WHEELS:
            try: profiles[action][wheel] = max(-255, min(255, int(values.get(wheel, profiles[action][wheel]))))
            except (TypeError, ValueError): pass
    return profiles

@dataclass
class Config:
    speed_mode: bool = False
    target_speed: float = 30.0
    kp: float = 2.0
    ki: float = 0.8
    kd: float = 0.05
    straight_pwm: int = 80
    pivot_pwm: int = 150
    curve_outer_pwm: int = 160
    curve_inner_pwm: int = 60
    servo_center_angle: int = 90
    servo_speed_dps: float = 45.0
    servo_acceleration_dps2: float = 120.0
    servo_qe_reversed: bool = True
    profiles: dict[str, dict[str, int]] = field(default_factory=default_profiles)


def legacy_scalar_profiles(config: Config) -> dict[str, dict[str, int]]:
    """Translate pre-profile PWM settings into the current action profiles."""
    straight = max(0, min(255, int(config.straight_pwm)))
    pivot = max(0, min(255, int(config.pivot_pwm)))
    outer = max(0, min(255, int(config.curve_outer_pwm)))
    inner = max(0, min(255, int(config.curve_inner_pwm)))
    profiles = default_profiles()
    profiles.update({
        "F": {wheel: straight for wheel in WHEELS},
        "B": {wheel: -straight for wheel in WHEELS},
        "PL": {"rf": pivot, "lf": -pivot, "lr": -pivot, "rr": pivot},
        "PR": {"rf": -pivot, "lf": pivot, "lr": pivot, "rr": -pivot},
        "FL": {"rf": outer, "lf": inner, "lr": inner, "rr": outer},
        "FR": {"rf": inner, "lf": outer, "lr": outer, "rr": inner},
        "BL": {"rf": -inner, "lf": -outer, "lr": -outer, "rr": -inner},
        "BR": {"rf": -outer, "lf": -inner, "lr": -inner, "rr": -outer},
    })
    return profiles


class RobotController:
    def __init__(self, port: str, config_path: Path | None = None, legacy_config_path: Path | None = None) -> None:
        default_path = Path(__file__).with_name("drive_config.json")
        self.port, self.config_path, self.lock = port, Path(config_path or default_path).expanduser(), threading.RLock()
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
        self.steering_direction = 0; self.last_steering_seen = 0.0
        self._last_sent_steering_direction: int | None = None
        self._last_steering_sent_at = 0.0
        self._last_sent_servo_limits: tuple[float, float] | None = None
        self._servo_target_angle: float | None = None
        self.imu = self.speed = self.ultrasonic = None; self.servo_angle: int | None = None; self.motor_output: list[int] | None = None; self.reply = self.error = ""; self.last_rx = 0.0
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
    def _write(self, command: str) -> None:
        self._connect()
        with self.serial_lock:
            if self.serial:
                try: self.serial.write((command + "\n").encode("ascii")); self.serial.flush()
                except Exception as exc: self.error = str(exc); self._close_serial()
    def select_action(self, action: str) -> str:
        action = action.upper()
        if action not in ACTIONS: raise ValueError("未知动作")
        with self.lock:
            # Compatibility endpoint for terminal/curl diagnostics.  The web
            # interface itself uses update_keys so releasing a key stops now.
            self.held_keys.clear(); self.action, self.last_client_seen = action, time.monotonic()
        return action
    @staticmethod
    def _action_from_keys(keys: set[str]) -> str:
        if "x" in keys: return "SPL"
        if "c" in keys: return "SPR"
        forward, turn = int("w" in keys) - int("s" in keys), int("d" in keys) - int("a" in keys)
        if forward > 0: return "FL" if turn < 0 else "FR" if turn > 0 else "F"
        if forward < 0: return "BL" if turn < 0 else "BR" if turn > 0 else "B"
        if turn < 0: return "PL"
        if turn > 0: return "PR"
        return "SF" if "r" in keys else "STOP"
    def update_keys(self, payload: dict) -> str:
        received = payload.get("keys", [])
        keys = {str(key).lower() for key in received if str(key).lower() in KEY_ACTIONS} if isinstance(received, list) else set()
        try: steering = int(payload.get("steering", 0))
        except (TypeError, ValueError): steering = 0
        steering = max(-1, min(1, steering))
        with self.lock:
            now = time.monotonic()
            self.held_keys, self.action, self.last_client_seen = keys, self._action_from_keys(keys), now
            self.steering_direction, self.last_steering_seen = steering, now
            return self.action
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
            self.held_keys.clear(); self.action = "STOP"; self.steering_direction = 0
        self._write("STOP")
    def set_servo_angle(self, raw_angle: object, *, fast: bool = False, track_target: bool = True) -> int:
        """Set a smooth target or request a mechanical-speed SG90 return."""
        try: angle = int(raw_angle)
        except (TypeError, ValueError): raise ValueError("舵机角度必须是 0 到 180 的整数") from None
        if not 0 <= angle <= 180: raise ValueError("舵机角度必须在 0 到 180 之间")
        self._write(f"SVF,{angle}" if fast else f"SV,{angle}")
        if self.error: raise RuntimeError(f"发送舵机命令失败：{self.error}")
        # Keep the web slider at the requested position immediately. The
        # Arduino's later OK:SV reply confirms the same value, but this is
        # intentionally not saved in drive_config.json.
        with self.lock:
            self.servo_angle = angle
            if track_target: self._servo_target_angle = float(angle)
        return angle
    def reconnect(self) -> dict:
        """Drop the current USB serial session and open it again.

        Opening an Arduino serial port may reset the board, so the method also
        puts the controller in STOP and waits in _connect for boot to finish.
        """
        with self.lock:
            self.held_keys.clear(); self.action, self.last_client_seen = "STOP", 0.0; self.steering_direction = 0
            self.last_rx = 0.0; self.error = ""
        self._close_serial(send_stop=True)
        self._connect()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if self.last_rx: break
            time.sleep(.05)
        return self.status()
    @staticmethod
    def _raw(action: str, cfg: Config) -> tuple[int, int, int, int]:
        if action == "STOP": return (0, 0, 0, 0)
        profile = cfg.profiles.get(action, default_profiles()[action])
        return tuple(profile[wheel] for wheel in WHEELS)
    @staticmethod
    def _speed(action: str, cfg: Config) -> tuple[float, float, int, int]:
        t, h = cfg.target_speed, cfg.target_speed * .5
        return {"F":(t,t,0,0),"SF":(t*.5,t*.5,0,0),"B":(-t,-t,0,0),"PL":(-t,t,-cfg.pivot_pwm,cfg.pivot_pwm),"PR":(t,-t,cfg.pivot_pwm,-cfg.pivot_pwm),"SPL":(-h,h,-cfg.pivot_pwm,cfg.pivot_pwm),"SPR":(h,-h,cfg.pivot_pwm,-cfg.pivot_pwm),"FL":(h,t,cfg.curve_inner_pwm,cfg.curve_outer_pwm),"FR":(t,h,cfg.curve_outer_pwm,cfg.curve_inner_pwm),"BL":(-t,-h,-cfg.curve_inner_pwm,-cfg.curve_outer_pwm),"BR":(-h,-t,-cfg.curve_outer_pwm,-cfg.curve_inner_pwm),"STOP":(0,0,0,0)}[action]

    def _sync_steering(self, now: float, cfg: Config) -> None:
        """Keep Arduino's local smooth steering state in sync with the browser."""
        with self.lock:
            direction = self.steering_direction if now - self.last_steering_seen <= CLIENT_TIMEOUT_SECONDS else 0
            if cfg.servo_qe_reversed:
                direction = -direction
            limits = (cfg.servo_speed_dps, cfg.servo_acceleration_dps2)
            send_limits = limits != self._last_sent_servo_limits
            send_direction = (
                direction != self._last_sent_steering_direction
                or (direction != 0 and now - self._last_steering_sent_at >= HEARTBEAT_SECONDS)
            )
            if send_limits:
                self._last_sent_servo_limits = limits
            if send_direction:
                self._last_sent_steering_direction = direction
                self._last_steering_sent_at = now
        if send_limits:
            self._write(f"SVC,{limits[0]:.1f},{limits[1]:.1f}")
        if send_direction:
            # Refresh a held direction at the browser heartbeat rate. Arduino
            # stops it smoothly if this heartbeat disappears.
            self._write(f"SVD,{direction}")
    def _parse(self, text: str) -> None:
        self.reply, self.last_rx = text, time.monotonic()
        try:
            parts = text.split(",")
            if text.startswith("IMU,") and len(parts) == 4: self.imu = [float(x) for x in parts[1:]]
            elif text.startswith("SPD,") and len(parts) == 7: self.speed = [float(x) for x in parts[1:]]
            elif (text.startswith("OK:M,") or text.startswith("OUT,")) and len(parts) == 5: self.motor_output = [int(x) for x in parts[1:]]
            elif text.startswith("US,") and len(parts) == 2:
                self.ultrasonic = float(parts[1])
            elif text.startswith("OK:SV,"):
                self.servo_angle = int(text.split(",", 1)[1])
            elif text in {"OK:STOP", "STATUS:STOPPED", "TIMEOUT:STOP"} or text.startswith("BLOCK:"):
                self.motor_output = [0, 0, 0, 0]
        except ValueError: pass
    def _run(self) -> None:
        next_motor, next_query = 0.0, 0.0
        while not self._stop.wait(SERVO_TICK_SECONDS):
            now = time.monotonic()
            with self.lock:
                action = self.action if now - self.last_client_seen <= CLIENT_TIMEOUT_SECONDS else "STOP"
                if action == "STOP": self.held_keys.clear(); self.action = "STOP"
                cfg = Config(**asdict(self.config))
            self._sync_steering(now, cfg)
            if now < next_motor:
                continue
            if cfg.speed_mode:
                left, right, front_left, front_right = self._speed(action, cfg)
                self._write(f"V,{left:.1f},{right:.1f}")
                self._write(f"M,{front_right},{front_left},0,0")
            else:
                self._write("M," + ",".join(map(str, self._raw(action, cfg))))
            next_motor = now + HEARTBEAT_SECONDS
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
        with self.lock: cfg, action, seen = asdict(self.config), self.action, self.last_client_seen
        serial_open = bool(self.serial and self.serial.is_open); age = time.monotonic() - self.last_rx if self.last_rx else None
        return {"serial":serial_open,"arduino_online":serial_open and age is not None and age <= 1.5,"last_rx_age":age,"error":self.error,"reply":self.reply,"config":cfg,"config_path":str(self.config_path),"config_source":self.config_source,"config_error":self.config_error,"action":action,"keys":sorted(self.held_keys),"client_online":time.monotonic()-seen<=CLIENT_TIMEOUT_SECONDS,"steering_direction":self.steering_direction,"imu":self.imu,"speed":self.speed,"ultrasonic":self.ultrasonic,"servo_angle":self.servo_angle,"motor_output":self.motor_output}
