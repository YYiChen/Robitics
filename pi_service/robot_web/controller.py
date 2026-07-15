"""Terminal-compatible action state machine and Arduino serial bridge."""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path

import serial

HEARTBEAT_SECONDS = 0.20
CLIENT_TIMEOUT_SECONDS = 0.80
ACTIONS = {"STOP", "F", "B", "PL", "PR", "FL", "FR", "BL", "BR"}
KEY_ACTIONS = {"q": "FL", "w": "F", "e": "FR", "a": "PL", "d": "PR", "z": "BL", "s": "B", "c": "BR"}
PROFILE_ACTIONS = ("F", "B", "PL", "PR", "FL", "FR", "BL", "BR")
WHEELS = ("rf", "lf", "lr", "rr")

def default_profiles() -> dict[str, dict[str, int]]:
    return {
        "F": {"rf": 80, "lf": 80, "lr": 80, "rr": 80}, "B": {"rf": -80, "lf": -80, "lr": -80, "rr": -80},
        "PL": {"rf": 150, "lf": -150, "lr": -150, "rr": 150}, "PR": {"rf": -150, "lf": 150, "lr": 150, "rr": -150},
        "FL": {"rf": 160, "lf": 60, "lr": 60, "rr": 160}, "FR": {"rf": 60, "lf": 160, "lr": 160, "rr": 60},
        "BL": {"rf": -60, "lf": -160, "lr": -160, "rr": -60}, "BR": {"rf": -160, "lf": -60, "lr": -60, "rr": -160},
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
    profiles: dict[str, dict[str, int]] = field(default_factory=default_profiles)

class RobotController:
    def __init__(self, port: str, config_path: Path | None = None) -> None:
        self.port, self.config_path, self.lock = port, config_path or Path(__file__).with_name("robot_config.json"), threading.RLock()
        self.config = self._load_config()
        self.serial_lock = threading.RLock()
        self.serial = None; self.action = "STOP"; self.held_keys: set[str] = set(); self.last_client_seen = 0.0
        self.imu = self.speed = self.ultrasonic = None; self.reply = self.error = ""; self.last_rx = 0.0
        self._stop = threading.Event()

    def _load_config(self) -> Config:
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8")); config = Config()
            for key in ("speed_mode", "target_speed", "kp", "ki", "kd", "straight_pwm", "pivot_pwm", "curve_outer_pwm", "curve_inner_pwm"):
                if key in data: setattr(config, key, type(getattr(config, key))(data[key]))
            config.profiles = normalize_profiles(data.get("profiles")); return config
        except (FileNotFoundError, json.JSONDecodeError, TypeError, ValueError, OSError): return Config()

    def _save_config(self) -> None:
        self.config_path.write_text(json.dumps(asdict(self.config), ensure_ascii=False, indent=2), encoding="utf-8")

    def start(self) -> None: threading.Thread(target=self._run, daemon=True, name="robot-control").start()
    def _connect(self) -> None:
        with self.serial_lock:
            if self.serial and self.serial.is_open: return
            try: self.serial = serial.Serial(self.port, 9600, timeout=.02, write_timeout=.5); time.sleep(2.5); self.error = ""
            except Exception as exc: self.serial, self.error = None, str(exc)
    def _close_serial(self, send_stop: bool = False) -> None:
        with self.serial_lock:
            port, self.serial = self.serial, None
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
        for key in ("q", "e", "z", "c"):
            if key in keys: return KEY_ACTIONS[key]
        forward, turn = int("w" in keys) - int("s" in keys), int("d" in keys) - int("a" in keys)
        if forward > 0: return "FL" if turn < 0 else "FR" if turn > 0 else "F"
        if forward < 0: return "BL" if turn < 0 else "BR" if turn > 0 else "B"
        return "PL" if turn < 0 else "PR" if turn > 0 else "STOP"
    def update_keys(self, payload: dict) -> str:
        received = payload.get("keys", [])
        keys = {str(key).lower() for key in received if str(key).lower() in KEY_ACTIONS} if isinstance(received, list) else set()
        with self.lock:
            self.held_keys, self.action, self.last_client_seen = keys, self._action_from_keys(keys), time.monotonic()
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
            self._save_config()
            return asdict(self.config)
    def stop_now(self) -> None:
        with self.lock: self.held_keys.clear(); self.action = "STOP"
        self._write("STOP")
    def reconnect(self) -> dict:
        """Drop the current USB serial session and open it again.

        Opening an Arduino serial port may reset the board, so the method also
        puts the controller in STOP and waits in _connect for boot to finish.
        """
        with self.lock:
            self.held_keys.clear(); self.action, self.last_client_seen = "STOP", 0.0
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
        return {"F":(t,t,0,0),"B":(-t,-t,0,0),"PL":(-t,t,-cfg.pivot_pwm,cfg.pivot_pwm),"PR":(t,-t,cfg.pivot_pwm,-cfg.pivot_pwm),"FL":(h,t,cfg.curve_inner_pwm,cfg.curve_outer_pwm),"FR":(t,h,cfg.curve_outer_pwm,cfg.curve_inner_pwm),"BL":(-t,-h,-cfg.curve_inner_pwm,-cfg.curve_outer_pwm),"BR":(-h,-t,-cfg.curve_outer_pwm,-cfg.curve_inner_pwm),"STOP":(0,0,0,0)}[action]
    def _parse(self, text: str) -> None:
        self.reply, self.last_rx = text, time.monotonic()
        try:
            parts = text.split(",")
            if text.startswith("IMU,") and len(parts) == 4: self.imu = [float(x) for x in parts[1:]]
            elif text.startswith("SPD,") and len(parts) == 7: self.speed = [float(x) for x in parts[1:]]
            elif text.startswith("US,") and len(parts) == 4:
                self.ultrasonic = [float(x) for x in parts[1:]]
            elif text.startswith("US,") and len(parts) == 2:
                # Older one-sensor firmware: US,<frontCm>
                self.ultrasonic = [-1.0, float(parts[1]), -1.0]
            elif text.startswith("US:FRONT="):
                # Periodic debug output from the one-sensor firmware.
                self.ultrasonic = [-1.0, float(text.split("=", 1)[1]), -1.0]
        except ValueError: pass
    def _run(self) -> None:
        next_query = 0.0
        while not self._stop.wait(HEARTBEAT_SECONDS):
            with self.lock:
                action = self.action if time.monotonic() - self.last_client_seen <= CLIENT_TIMEOUT_SECONDS else "STOP"
                if action == "STOP": self.held_keys.clear(); self.action = "STOP"
                cfg = Config(**asdict(self.config))
            if cfg.speed_mode:
                left, right, front_left, front_right = self._speed(action, cfg)
                self._write(f"V,{left:.1f},{right:.1f}")
                self._write(f"M,{front_right},{front_left},0,0")
            else:
                self._write("M," + ",".join(map(str, self._raw(action, cfg))))
            with self.serial_lock:
                port = self.serial
                if port:
                    for _ in range(8):
                        line = port.readline().decode("ascii", "ignore").strip()
                        if line: self._parse(line)
            if time.monotonic() >= next_query:
                for command in ("IMU", "SPD", "US"): self._write(command)
                next_query = time.monotonic() + .5
    def status(self) -> dict:
        with self.lock: cfg, action, seen = asdict(self.config), self.action, self.last_client_seen
        serial_open = bool(self.serial and self.serial.is_open); age = time.monotonic() - self.last_rx if self.last_rx else None
        return {"serial":serial_open,"arduino_online":serial_open and age is not None and age <= 1.5,"last_rx_age":age,"error":self.error,"reply":self.reply,"config":cfg,"action":action,"keys":sorted(self.held_keys),"client_online":time.monotonic()-seen<=CLIENT_TIMEOUT_SECONDS,"imu":self.imu,"speed":self.speed,"ultrasonic":self.ultrasonic}
