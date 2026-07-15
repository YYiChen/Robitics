"""Terminal-compatible action state machine and Arduino serial bridge."""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass

import serial

HEARTBEAT_SECONDS = 0.20
CLIENT_TIMEOUT_SECONDS = 0.80
ACTIONS = {"STOP", "F", "B", "PL", "PR", "FL", "FR", "BL", "BR"}
KEY_ACTIONS = {"q": "FL", "w": "F", "e": "FR", "a": "PL", "d": "PR", "z": "BL", "s": "B", "c": "BR"}

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

class RobotController:
    def __init__(self, port: str) -> None:
        self.port, self.config, self.lock = port, Config(), threading.RLock()
        self.serial = None; self.action = "STOP"; self.held_keys: set[str] = set(); self.last_client_seen = 0.0
        self.imu = self.speed = self.ultrasonic = None; self.reply = self.error = ""
        self._stop = threading.Event()

    def start(self) -> None: threading.Thread(target=self._run, daemon=True, name="robot-control").start()
    def _connect(self) -> None:
        if self.serial and self.serial.is_open: return
        try: self.serial = serial.Serial(self.port, 9600, timeout=.02, write_timeout=.5); time.sleep(2.5); self.error = ""
        except Exception as exc: self.serial, self.error = None, str(exc)
    def _write(self, command: str) -> None:
        self._connect()
        if self.serial:
            try: self.serial.write((command + "\n").encode("ascii")); self.serial.flush()
            except Exception as exc: self.error = str(exc); self.serial = None
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
                if key in payload: setattr(self.config, key, bool(payload[key]) if isinstance(old, bool) else type(old)(payload[key]))
            self.config.target_speed = max(0.0, min(200.0, self.config.target_speed))
            for key in ("straight_pwm", "pivot_pwm", "curve_outer_pwm", "curve_inner_pwm"):
                setattr(self.config, key, max(0, min(255, getattr(self.config, key))))
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
            previous, self.serial = self.serial, None
            self.error = ""
        if previous:
            try:
                if previous.is_open:
                    previous.write(b"STOP\n")
                    previous.flush()
                    previous.close()
            except Exception:
                # The point of reconnect is recovery, so a failed close is safe
                # to ignore before opening a new port session.
                pass
        self._connect()
        return self.status()
    @staticmethod
    def _raw(action: str, cfg: Config) -> tuple[int, int, int, int]:
        s, p, outer, inner = cfg.straight_pwm, cfg.pivot_pwm, cfg.curve_outer_pwm, cfg.curve_inner_pwm
        return {"F":(s,s,s,s),"B":(-s,-s,-s,-s),"PL":(p,-p,-p,p),"PR":(-p,p,p,-p),"FL":(outer,inner,inner,outer),"FR":(inner,outer,outer,inner),"BL":(-inner,-outer,-outer,-inner),"BR":(-outer,-inner,-inner,-outer),"STOP":(0,0,0,0)}[action]
    @staticmethod
    def _speed(action: str, cfg: Config) -> tuple[float, float, int, int]:
        t, h = cfg.target_speed, cfg.target_speed * .5
        return {"F":(t,t,0,0),"B":(-t,-t,0,0),"PL":(-t,t,-cfg.pivot_pwm,cfg.pivot_pwm),"PR":(t,-t,cfg.pivot_pwm,-cfg.pivot_pwm),"FL":(h,t,cfg.curve_inner_pwm,cfg.curve_outer_pwm),"FR":(t,h,cfg.curve_outer_pwm,cfg.curve_inner_pwm),"BL":(-t,-h,-cfg.curve_inner_pwm,-cfg.curve_outer_pwm),"BR":(-h,-t,-cfg.curve_outer_pwm,-cfg.curve_inner_pwm),"STOP":(0,0,0,0)}[action]
    def _parse(self, text: str) -> None:
        self.reply = text
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
            if self.serial:
                for _ in range(8):
                    line = self.serial.readline().decode("ascii", "ignore").strip()
                    if line: self._parse(line)
            if time.monotonic() >= next_query:
                for command in ("IMU", "SPD", "US"): self._write(command)
                next_query = time.monotonic() + .5
    def status(self) -> dict:
        with self.lock: cfg, action, seen = asdict(self.config), self.action, self.last_client_seen
        return {"serial":bool(self.serial and self.serial.is_open),"error":self.error,"reply":self.reply,"config":cfg,"action":action,"keys":sorted(self.held_keys),"client_online":time.monotonic()-seen<=CLIENT_TIMEOUT_SECONDS,"imu":self.imu,"speed":self.speed,"ultrasonic":self.ultrasonic}
