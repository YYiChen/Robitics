"""Serial protocol owner. Arduino remains the only motor/PID authority."""
from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass

import serial

@dataclass
class Config:
    speed_mode: bool = False
    target_speed: float = 30.0
    kp: float = 2.0
    ki: float = 0.8
    kd: float = 0.05
    pwm: int = 80

class RobotController:
    def __init__(self, port: str) -> None:
        self.port, self.config, self.lock = port, Config(), threading.RLock()
        self.serial = None; self.keys: set[str] = set(); self.last_heartbeat = 0.0
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
    def update_keys(self, payload: dict) -> None:
        with self.lock: self.keys = {str(k).lower() for k in payload.get("keys", [])}; self.last_heartbeat = time.monotonic()
    def update_config(self, payload: dict) -> dict:
        with self.lock:
            for key in asdict(self.config):
                if key in payload: setattr(self.config, key, type(getattr(self.config, key))(payload[key]))
            self.config.target_speed = max(0.0, min(200.0, self.config.target_speed)); self.config.pwm = max(0, min(255, self.config.pwm))
            return asdict(self.config)
    def stop_now(self) -> None:
        with self.lock: self.keys.clear()
        self._write("STOP")
    def _motors(self) -> tuple[int, int, int, int]:
        with self.lock: keys, pwm = set(self.keys), self.config.pwm
        f, turn = int("w" in keys)-int("s" in keys), int("d" in keys)-int("a" in keys)
        if not f and not turn: return (0,0,0,0)
        left, right = (f*pwm, f*pwm) if f else (-turn*pwm, turn*pwm)
        return (right,left,left,right)
    def _parse(self, text: str) -> None:
        self.reply = text
        try:
            parts = text.split(",")
            if text.startswith("IMU,") and len(parts) == 4: self.imu = [float(x) for x in parts[1:]]
            elif text.startswith("SPD,") and len(parts) == 7: self.speed = [float(x) for x in parts[1:]]
            elif text.startswith("US,") and len(parts) == 4: self.ultrasonic = [float(x) for x in parts[1:]]
        except ValueError: pass
    def _run(self) -> None:
        next_query = 0.0
        while not self._stop.wait(.2):
            with self.lock: timed_out = time.monotonic()-self.last_heartbeat > .8; cfg = Config(**asdict(self.config))
            motors = (0,0,0,0) if timed_out else self._motors()
            self._write("M," + ",".join(map(str, motors)))
            if cfg.speed_mode:
                self._write(f"KP,{cfg.kp:.3f}"); self._write(f"KI,{cfg.ki:.3f}"); self._write(f"KD,{cfg.kd:.3f}")
                self._write(f"V,{cfg.target_speed if motors[2] >= 0 else -cfg.target_speed:.1f},{cfg.target_speed if motors[3] >= 0 else -cfg.target_speed:.1f}")
            if self.serial:
                for _ in range(8):
                    line = self.serial.readline().decode("ascii", "ignore").strip()
                    if line: self._parse(line)
            if time.monotonic() >= next_query:
                for command in ("IMU", "SPD", "US"): self._write(command)
                next_query = time.monotonic() + .5
    def status(self) -> dict:
        with self.lock: cfg, keys = asdict(self.config), sorted(self.keys)
        return {"serial": bool(self.serial and self.serial.is_open), "error":self.error, "reply":self.reply, "config":cfg, "keys":keys, "imu":self.imu, "speed":self.speed, "ultrasonic":self.ultrasonic}
