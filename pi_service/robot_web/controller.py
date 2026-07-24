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
CARD_COMMAND_ACK_TIMEOUT_SECONDS = 1.20
ACTIONS = {"STOP", "F", "SF", "B", "PL", "PR", "SPL", "SPR", "FL", "FR", "BL", "BR"}
# Q/E are steering-servo controls rather than motor profiles.
KEY_ACTIONS = {"w": "F", "slow": "SF", "a": "PL", "d": "PR", "s": "B", "x": "SPL", "c": "SPR"}
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
        self.imu = self.speed = self.ultrasonic = None; self.servo_angle: int | None = None; self.motor_output: list[int] | None = None; self.card_feed_state = self.card_deal_state = "idle"; self.card_motor_protocol = "unknown"; self.card_command_reply = ""; self._card_reply_sequence = 0; self._card_reply_waiting_for = None
        self._card_reply_condition = threading.Condition(self.lock)
        self._card_command_lock = threading.Lock()
        self._deal_request_lock = threading.Lock()
        self._last_deal_request_token = ""
        self._last_deal_request_result: dict | None = None
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
        return "SF" if "slow" in keys else "STOP"
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
    @staticmethod
    def _timed_motor_parameters(raw_pwm: object, raw_duration_ms: object) -> tuple[int, int]:
        try:
            pwm = int(raw_pwm)
            duration_ms = int(raw_duration_ms)
        except (TypeError, ValueError):
            raise ValueError("电机功率和运行时间必须是整数") from None
        if not 1 <= pwm <= 255:
            raise ValueError("电机 PWM 必须在 1 到 255 之间；0 不会驱动电机")
        if not 100 <= duration_ms <= 60000:
            raise ValueError("电机运行时间必须在 100 到 60000 毫秒之间")
        return pwm, duration_ms
    def _send_card_command_and_wait(self, motor: str, command: str) -> str:
        with self._card_command_lock:
            with self._card_reply_condition:
                before = self._card_reply_sequence
                self._card_reply_waiting_for = motor
            if not self._write(command):
                with self._card_reply_condition:
                    self._card_reply_waiting_for = None
                raise RuntimeError(f"串口写入失败：{self.error or 'Arduino 串口未打开'}")
            deadline = time.monotonic() + CARD_COMMAND_ACK_TIMEOUT_SECONDS
            with self._card_reply_condition:
                while self._card_reply_sequence == before:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        self._card_reply_waiting_for = None
                        raise TimeoutError(f"Arduino 未确认 {command}")
                    self._card_reply_condition.wait(remaining)
                self._card_reply_waiting_for = None
                return self.card_command_reply
    @staticmethod
    def _card_result_state(reply: str) -> str:
        if reply.startswith("OK:"): return "running"
        if reply.startswith("BUSY:"): return "busy"
        raise RuntimeError(f"Arduino 拒绝卡牌电机命令：{reply}")
    def deal_card(self, raw_pwm: object = 255, raw_duration_ms: object = 1000) -> str:
        """Request one M4 cycle and wait for Arduino acknowledgement."""
        pwm, duration_ms = self._timed_motor_parameters(raw_pwm, raw_duration_ms)
        with self.lock:
            self.card_deal_state = "requested"
            protocol = self.card_motor_protocol
        command = "DEAL" if protocol == "legacy" else f"DEAL,{pwm},{duration_ms}"
        try:
            reply = self._send_card_command_and_wait("DEAL", command)
            state = self._card_result_state(reply)
        except (TimeoutError, RuntimeError) as first_error:
            if protocol != "unknown":
                with self.lock:
                    self.card_deal_state = "error"
                raise RuntimeError(f"发送出牌命令失败：{first_error}") from first_error
            # A firmware variant whose READY line was missed may only support
            # the old exact DEAL command. Probe it once before reporting fail.
            try:
                reply = self._send_card_command_and_wait("DEAL", "DEAL")
                state = self._card_result_state(reply)
                with self.lock:
                    if reply == "OK:DEAL":
                        self.card_motor_protocol = "legacy"
                    elif reply.startswith("OK:DEAL,"):
                        self.card_motor_protocol = "adjustable"
                    protocol = self.card_motor_protocol
            except (TimeoutError, RuntimeError) as fallback_error:
                with self.lock:
                    self.card_deal_state = "error"
                raise RuntimeError(f"新版和旧版出牌命令均未被 Arduino 接受：{fallback_error}") from fallback_error
        with self.lock:
            self.card_deal_state = "running" if state in {"running", "busy"} else state
        return "legacy" if protocol == "legacy" and state == "running" else state
    def feed_cards(self, raw_pwm: object = 255, raw_duration_ms: object = 5000) -> str:
        """Request one adjustable Arduino-timed M3 feed cycle."""
        pwm, duration_ms = self._timed_motor_parameters(raw_pwm, raw_duration_ms)
        with self.lock:
            protocol = self.card_motor_protocol
        if protocol == "legacy":
            raise RuntimeError("Arduino 固件过旧，不支持网页触发 M3；请重新烧录新版 motor_bridge 固件")
        with self.lock:
            self.card_feed_state = "requested"
        try:
            reply = self._send_card_command_and_wait("FEED", f"FEED,{pwm},{duration_ms}")
            state = self._card_result_state(reply)
        except (TimeoutError, RuntimeError) as exc:
            with self.lock:
                self.card_feed_state = "error"
            raise RuntimeError(f"发送送牌命令失败：{exc}") from exc
        with self.lock:
            self.card_feed_state = "running" if state in {"running", "busy"} else state
        return state
    def deal_from_key_request(self, request: object) -> dict | None:
        if not isinstance(request, dict):
            return None
        token = str(request.get("token", "")).strip()
        if not token:
            raise ValueError("出牌事件缺少 token")
        feed_pwm, feed_duration_ms = self._timed_motor_parameters(
            request.get("feed_pwm", 255),
            request.get("feed_duration_ms", 5000),
        )
        deal_pwm, deal_duration_ms = self._timed_motor_parameters(
            request.get("deal_pwm", request.get("pwm", 255)),
            request.get("deal_duration_ms", request.get("duration_ms", 1000)),
        )
        with self._deal_request_lock:
            if token == self._last_deal_request_token:
                return self._last_deal_request_result
            self._last_deal_request_token = token
            self._last_deal_request_result = {"token": token, "state": "pending", "reply": ""}
            threading.Thread(
                target=self._complete_deal_request,
                args=(token, feed_pwm, feed_duration_ms, deal_pwm, deal_duration_ms),
                daemon=True,
                name="card-deal-request",
            ).start()
            return self._last_deal_request_result
    def _complete_deal_request(
        self,
        token: str,
        feed_pwm: int,
        feed_duration_ms: int,
        deal_pwm: int,
        deal_duration_ms: int,
    ) -> None:
        feed_result: dict
        deal_result: dict
        try:
            feed_state = self.feed_cards(feed_pwm, feed_duration_ms)
            feed_result = {"state": feed_state, "reply": self.card_command_reply}
        except (RuntimeError, ValueError) as exc:
            feed_result = {"state": "error", "reply": self.card_command_reply, "error": str(exc)}
        try:
            deal_state = self.deal_card(deal_pwm, deal_duration_ms)
            deal_result = {"state": deal_state, "reply": self.card_command_reply}
        except (RuntimeError, ValueError) as exc:
            deal_result = {"state": "error", "reply": self.card_command_reply, "error": str(exc)}
        failed = sum(item["state"] == "error" for item in (feed_result, deal_result))
        result = {
            "token": token,
            "state": "error" if failed == 2 else "partial" if failed == 1 else "running",
            "feed": feed_result,
            "deal": deal_result,
        }
        with self._deal_request_lock:
            if token == self._last_deal_request_token:
                self._last_deal_request_result = result
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
        # M1 is right drive and M2 is left drive. M3/M4 are card motors and
        # are controlled exclusively by Arduino's adjustable timed commands.
        return (profile["rf"], profile["lf"], 0, 0)
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
        try:
            parts = text.split(",")
            if text.startswith("READY:MOTOR_BRIDGE"):
                if "DEAL_ADJUSTABLE" in text:
                    self.card_motor_protocol = "adjustable"
                elif "DEAL_1000MS" in text:
                    self.card_motor_protocol = "legacy"
            elif text.startswith("IMU,") and len(parts) == 4: self.imu = [float(x) for x in parts[1:]]
            elif text.startswith("SPD,") and len(parts) == 7: self.speed = [float(x) for x in parts[1:]]
            elif (text.startswith("OK:M,") or text.startswith("OUT,")) and len(parts) == 5: self.motor_output = [int(x) for x in parts[1:]]
            elif text.startswith("US,") and len(parts) == 2:
                self.ultrasonic = float(parts[1])
            elif text.startswith("OK:SV,"):
                self.servo_angle = int(text.split(",", 1)[1])
            elif text.startswith("SVP,"):
                self.servo_angle = int(text.split(",", 1)[1])
            elif text.startswith("OK:DEAL") or text == "BUSY:DEAL":
                self.card_deal_state = "running"
            elif text == "DEAL:DONE":
                self.card_deal_state = "idle"
            elif text.startswith("OK:FEED") or text == "BUSY:FEED":
                self.card_feed_state = "running"
            elif text == "FEED:DONE":
                self.card_feed_state = "idle"
            elif text in {"OK:STOP", "STATUS:STOPPED", "STATUS:DRIVE_STOPPED", "TIMEOUT:STOP"} or text.startswith("BLOCK:"):
                outputs = list(self.motor_output or [0, 0, 0, 0])
                outputs[0] = outputs[1] = 0
                self.motor_output = outputs
        except ValueError: pass
    def _run(self) -> None:
        next_motor, next_query, next_servo_query = 0.0, 0.0, 0.0
        while not self._stop.wait(SERVO_TICK_SECONDS):
            now = time.monotonic()
            with self.lock:
                action = self.action if now - self.last_client_seen <= CLIENT_TIMEOUT_SECONDS else "STOP"
                if action == "STOP": self.held_keys.clear(); self.action = "STOP"
                cfg = Config(**asdict(self.config))
            self._sync_steering(now, cfg)
            if now >= next_servo_query:
                self._write("SVP")
                next_servo_query = now + HEARTBEAT_SECONDS
            if now < next_motor:
                continue
            if cfg.speed_mode:
                left, right, front_left, front_right = self._speed(action, cfg)
                self._write(f"V,{left:.1f},{right:.1f}")
                # The Arduino applies right/left PID to M1/M2 and ignores the
                # M3/M4 fields because those belong to the card mechanism.
                self._write("M,0,0,0,0")
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
        with self.lock: cfg, action, seen, feed_state, deal_state, card_protocol, card_reply = asdict(self.config), self.action, self.last_client_seen, self.card_feed_state, self.card_deal_state, self.card_motor_protocol, self.card_command_reply
        serial_open = bool(self.serial and self.serial.is_open); age = time.monotonic() - self.last_rx if self.last_rx else None
        return {"serial":serial_open,"arduino_online":serial_open and age is not None and age <= 1.5,"last_rx_age":age,"error":self.error,"reply":self.reply,"config":cfg,"config_path":str(self.config_path),"config_source":self.config_source,"config_error":self.config_error,"action":action,"keys":sorted(self.held_keys),"client_online":time.monotonic()-seen<=CLIENT_TIMEOUT_SECONDS,"steering_direction":self.steering_direction,"imu":self.imu,"speed":self.speed,"ultrasonic":self.ultrasonic,"servo_angle":self.servo_angle,"motor_output":self.motor_output,"card_feed_state":feed_state,"card_deal_state":deal_state,"card_motor_protocol":card_protocol,"card_command_reply":card_reply}
