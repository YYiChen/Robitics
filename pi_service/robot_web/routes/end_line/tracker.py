"""Port-5000 adaptor with red-line-closed-loop keyboard turning."""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

from .line_following import FastLineConfig, analyse_fast_line, pwm_for_line
from .perception import EndLineConfig, EndLineStopPlanner, RedEndBandDetector
from .return_route import ReturnRouteRecorder, blend_return_pwm
from .turn_profiles import TurnProfile, load_turn_profile, save_turn_profile


SERVICE_ROOT = Path(__file__).resolve().parents[3]
LEGACY_EXPERIMENT = SERVICE_ROOT / "experiments" / "end_line_turn_validation"
TUNING_PATH = LEGACY_EXPERIMENT / "end_line_web_tuning.json"
TURN_90_PATH = LEGACY_EXPERIMENT / "turn_90.json"
TURN_180_PATH = LEGACY_EXPERIMENT / "turn_180.json"
RETURN_ROUTE_PATH = SERVICE_ROOT / "logs" / "end_line_return_route.json"
TUNING_RULES = {
    "process_fps": (float, 5.0, 60.0),
    "straight_pwm": (int, 0, 255),
    "correction_deadband": (float, 0.0, 1.0),
    "correction_gain": (float, 0.0, 1000.0),
    "minimum_correction_pwm": (int, 0, 255),
    "maximum_correction_pwm": (int, 0, 255),
    "green_hue_min": (int, 0, 179),
    "green_hue_max": (int, 0, 179),
    "green_saturation_min": (int, 0, 255),
    "green_dilate_radius_px": (int, 1, 100),
    "green_support_inner_px": (int, 0, 100),
    "green_support_outer_px": (int, 1, 150),
    "green_support_min_ratio": (float, 0.0, 1.0),
    "red_channel_min": (int, 0, 255),
    "red_excess_min": (int, 0, 255),
    "red_roi_top_ratio": (float, 0.0, .8),
    "red_roi_side_ratio": (float, 0.0, .45),
    "red_min_component_area": (int, 1, 100000),
    "red_min_span_ratio": (float, .01, 1.0),
    "line_lost_confirm_frames": (int, 1, 20),
    "red_direction_memory_frames": (int, 1, 300),
    "brake_hold_seconds": (float, 0.0, 3.0),
    "turn_90_pwm": (int, 0, 255),
    "turn_90_step_seconds": (float, .05, 20.0),
    "turn_180_pwm": (int, 0, 255),
    "turn_180_step_seconds": (float, .05, 20.0),
    "turn_interstep_pause_seconds": (float, 0.0, 10.0),
    "red_alignment_min_angle": (float, 45.0, 90.0),
    "red_alignment_confirm_frames": (int, 1, 10),
    "turn_90_max_steps": (int, 1, 12),
    "turn_180_max_steps": (int, 1, 24),
    "return_replay_weight": (float, 0.0, .5),
    "return_line_lost_confirm_frames": (int, 1, 20),
    "return_turn_pause_seconds": (float, .05, 3.0),
}
FACE_TURN_HEARTBEAT_SECONDS = 3.0
# J/L is an operator-triggered, camera-stopped pivot.  It needs enough torque
# to overcome the vehicle's static wheel friction, independently of the short
# duration used by the Q/E preset turn.
FACE_TURN_PWM = 255
FACE_TURN_PULSE_SECONDS = .20
FACE_TURN_COOLDOWN_SECONDS = 2.0
FACE_TURN_MAX_SECONDS = 15.0
FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED = .10
FACE_TURN_LINE_CENTER_CONFIRM_FRAMES = 3


class EndLineTurnAdaptorRouteTracker:
    route_mode = "end_line_turn_adaptor"

    def __init__(
        self,
        controller,
        camera,
        publisher,
        gate,
        tuning_path: Path = TUNING_PATH,
        config_store=None,
        return_recorder: ReturnRouteRecorder | None = None,
    ) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        self._stop, self._thread, self._lock, self._tuning_lock = threading.Event(), None, threading.RLock(), threading.RLock()
        self._last_center_x, self._motor_active = None, False
        self._tuning_path = tuning_path
        self.config_store = config_store
        (self._process_fps, self._straight_pwm, self._fast_config, self._line_config,
         self._turn_interstep_pause_seconds, self._red_alignment_min_angle,
         self._red_alignment_confirm_frames, self._turn_90_max_steps,
         self._turn_180_max_steps, self._return_replay_weight,
         self._return_line_lost_confirm_frames,
         self._return_turn_pause_seconds) = self._load_tuning()
        if self.config_store is not None:
            stored = self._read_tuning_data()
            self._turn_90 = TurnProfile(int(stored.get("turn_90_pwm", 200)), float(stored.get("turn_90_step_seconds", 1.25)))
            self._turn_180 = TurnProfile(int(stored.get("turn_180_pwm", 200)), float(stored.get("turn_180_step_seconds", 1.25)))
        else:
            self._turn_90 = load_turn_profile(TURN_90_PATH, TurnProfile(200, 1.25), steps=2)
            self._turn_180 = load_turn_profile(TURN_180_PATH, TurnProfile(200, 1.25), steps=4)
        self._planner = EndLineStopPlanner(self._line_config)
        self._red_detector = RedEndBandDetector(self._line_config)
        self._last_red_side, self._last_red_seen_frame = None, -10_000
        self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
        self._manual_degrees, self._manual_profile = None, None
        self._manual_max_steps, self._manual_steps_started = 0, 0
        self._red_alignment_streak = 0
        self._face_turn_side, self._face_turn_deadline = None, 0.0
        self._face_turn_started = self._face_turn_phase_until = 0.0
        self._face_turn_pulse_active = False
        self._face_turn_line_departed = False
        self._face_turn_line_center_streak = 0
        self._return_recorder = return_recorder or ReturnRouteRecorder(
            RETURN_ROUTE_PATH,
            nominal_sample_seconds=1.0 / self._process_fps,
        )
        self._return_replay = None
        self._return_turn_phase_until = 0.0
        self._return_turn_pulse_active = False
        self._return_turn_steps = 0
        self._return_line_departed = False
        self._return_line_center_streak = 0
        self._return_line_lost_frames = 0
        self._pending_return_checkpoints: list[str] = []
        if hasattr(self.controller, "add_card_event_listener"):
            self.controller.add_card_event_listener(self._on_card_event)
        # This deployment is deliberately keyboard-only: M arms turn commands
        # but must never make the vehicle start following the white line.
        self._manual_only = True
        self._run_log = None
        self._status = {"available": True, "running": False, "enabled": False, "mode": self.route_mode, "state": "starting", "detail": "单白线红终点 adaptor 启动中", "frame": 0, "confidence": 0.0}

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="end-line-turn-adaptor")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._stop_motor()
        if hasattr(self.controller, "remove_card_event_listener"):
            self.controller.remove_card_event_listener(self._on_card_event)
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _stop_motor(self) -> None:
        if self._motor_active:
            self.controller.stop_now()
        self._motor_active = False

    def _set_status(self, **changes) -> None:
        with self._lock:
            self._status.update(changes)

    def status_dict(self) -> dict:
        with self._lock:
            result = dict(self._status)
        tuning_path = (
            f"{self.config_store.local_path}#routes.end_line"
            if self.config_store is not None
            else str(self._tuning_path)
        )
        result.update({
            "enabled": self.gate.enabled(),
            "tuning": self._tuning_values(),
            "tuning_path": tuning_path,
            "return_route": self._return_recorder.status_dict(),
            "return_replay": (
                None if self._return_replay is None
                else self._return_replay.status_dict()
            ),
        })
        return result

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled:
            self._stop_motor()
            with self._tuning_lock:
                self._planner.reset()
            self._last_red_side, self._last_red_seen_frame = None, -10_000
            self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
            self._manual_max_steps, self._manual_steps_started, self._red_alignment_streak = 0, 0, 0
            self._face_turn_side, self._face_turn_deadline = None, 0.0
            self._face_turn_started = self._face_turn_phase_until = 0.0
            self._face_turn_pulse_active = False
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._return_replay = None
            self._return_turn_pulse_active = False
            self._return_line_departed = False
            self._return_line_center_streak = 0
            self._return_line_lost_frames = 0
        self._set_status(enabled=enabled, detail="按键转向已解锁，等待 Q/E/U/I" if enabled else "已暂停，电机已停止")
        return self.status_dict()

    def request_follow_to_end(self) -> dict:
        """N key: follow white line until it ends, then stop and return to manual."""
        if not self.gate.enabled():
            raise ValueError("请先按 M 开启自动电机门控，再触发 N 巡线")
        if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE"}:
            raise ValueError("当前已有转向动作，请等待其完成或按 M 停止")
        self._manual_only = False
        self._return_replay = None
        self._return_recorder.start_recording()
        self._planner.reset()
        self._last_red_side, self._last_red_seen_frame = None, -10_000
        self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
        self._set_status(state="FOLLOWING_TO_END", detail="N 已触发，沿白线行驶至尽头")
        return self.status_dict()

    def request_return(self) -> dict:
        """R key: turn around, then return under vision with replay feed-forward."""
        if not self.gate.enabled():
            raise ValueError("请先按 M 开启自动电机门控，再触发 R 返程")
        if not self._return_recorder.has_samples():
            raise ValueError("尚未记录有效去程，不能开始返程")
        if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE"}:
            raise ValueError("当前已有转向动作，请等待完成或按 M 停止")
        self._stop_motor()
        self._flush_return_checkpoints()
        self._return_replay = self._return_recorder.prepare_return()
        self._manual_only = False
        self._planner.reset()
        self._return_turn_phase_until = time.monotonic() + self._turn_180.step_seconds
        self._return_turn_pulse_active = True
        self._return_turn_steps = 1
        self._return_line_departed = False
        self._return_line_center_streak = 0
        self._return_line_lost_frames = 0
        self._motion_phase = "RETURN_TURN"
        self._set_status(
            state="RETURN_TURN",
            detail="R 已触发：原地掉头，重新捕获白线后按分段记录返程",
        )
        return self.status_dict()

    def _on_card_event(self, event: str) -> None:
        if event == "DEAL:DONE":
            # Serial RX only enqueues the landmark.  The vision worker owns
            # route persistence so a growing JSON file cannot delay heartbeats.
            with self._lock:
                self._pending_return_checkpoints.append(
                    f"deal_complete_{int(time.time() * 1000)}"
                )

    def _flush_return_checkpoints(self) -> None:
        with self._lock:
            checkpoints = tuple(self._pending_return_checkpoints)
            self._pending_return_checkpoints.clear()
        for checkpoint in checkpoints:
            self._return_recorder.checkpoint(checkpoint)

    def request_manual_turn(self, command: str) -> dict:
        """M-gated turn; profiles are refreshed from disk for every key press."""
        # The JSON files are the source of truth.  Reload here as well as on
        # web save so a profile copied/edited on the Pi is effective on the
        # very next Q/E/U/I press without restarting the web process.
        with self._tuning_lock:
            if self.config_store is not None:
                stored = self._read_tuning_data()
                self._turn_90 = TurnProfile(int(stored.get("turn_90_pwm", self._turn_90.pwm)), float(stored.get("turn_90_step_seconds", self._turn_90.step_seconds)))
                self._turn_180 = TurnProfile(int(stored.get("turn_180_pwm", self._turn_180.pwm)), float(stored.get("turn_180_step_seconds", self._turn_180.step_seconds)))
            else:
                self._turn_90 = load_turn_profile(TURN_90_PATH, self._turn_90, steps=2)
                self._turn_180 = load_turn_profile(TURN_180_PATH, self._turn_180, steps=4)
            commands = {"LEFT_90": ("LEFT", 90, self._turn_90, self._turn_90_max_steps), "RIGHT_90": ("RIGHT", 90, self._turn_90, self._turn_90_max_steps), "LEFT_180": ("LEFT", 180, self._turn_180, self._turn_180_max_steps), "RIGHT_180": ("RIGHT", 180, self._turn_180, self._turn_180_max_steps)}
        try:
            side, degrees, profile, steps = commands[str(command).upper()]
        except KeyError as exc:
            raise ValueError("手动转向只支持 LEFT_90、RIGHT_90、LEFT_180、RIGHT_180") from exc
        if not self.gate.enabled():
            raise ValueError("请先按 M 开启自动电机门控，再触发转向")
        if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE"}:
            raise ValueError("当前已有转向动作，请等待其完成或按 M 停止")
        self._stop_motor()
        self._pending_turn_side, self._manual_degrees, self._manual_profile = side, degrees, profile
        self._manual_max_steps, self._manual_steps_started, self._red_alignment_streak = steps, 1, 0
        self._motion_phase, self._action_until = "MANUAL_STEP", time.monotonic() + profile.step_seconds
        self._set_status(state="MANUAL_STEP", detail=f"{side} {degrees}° red-calibrated pulse 1/{steps}", manual_turn=f"{side}_{degrees}")
        return self.status_dict()

    def request_face_center_turn(self, command: str) -> dict:
        """PC face-centering control with a Pi-local dead-man timeout.

        START_LEFT/START_RIGHT select a pivot direction.  The PC must keep
        sending HEARTBEAT while it is analysing frames; STOP is used only after
        its own MediaPipe detector confirms the face is centred.  A missing PC
        heartbeat never leaves the motor latched.
        """
        command = str(command).upper()
        if command in {"START_LEFT", "START_RIGHT"}:
            if not self.gate.enabled():
                raise ValueError("请先按 M 开启自动电机门控，再启动人脸居中转向")
            if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE", "FACE_CENTER_TURN"}:
                raise ValueError("当前已有其他转向动作，请等待完成或按 M 停止")
            self._manual_only = True
            self._stop_motor()
            now = time.monotonic()
            self._face_turn_side = "LEFT" if command.endswith("LEFT") else "RIGHT"
            self._face_turn_deadline = now + FACE_TURN_HEARTBEAT_SECONDS
            self._face_turn_started = now
            self._face_turn_phase_until = now + FACE_TURN_PULSE_SECONDS
            self._face_turn_pulse_active = True
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._motion_phase = "FACE_CENTER_TURN"
            self._set_status(state="FACE_CENTER_TURN", detail=f"PC 人脸居中：脉冲原地{self._face_turn_side}转，等待心跳", face_turn_active=True, face_search_side=self._face_turn_side)
        elif command == "HEARTBEAT":
            if self._motion_phase != "FACE_CENTER_TURN" or self._face_turn_side is None:
                raise ValueError("当前没有进行中的人脸居中转向")
            self._face_turn_deadline = time.monotonic() + FACE_TURN_HEARTBEAT_SECONDS
        elif command == "STOP":
            self._stop_motor()
            self._motion_phase, self._face_turn_side, self._face_turn_deadline = "MANUAL_COMPLETE", None, 0.0
            self._face_turn_started = self._face_turn_phase_until = 0.0
            self._face_turn_pulse_active = False
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._set_status(state="FACE_CENTERED_STOP", detail="PC 已确认人脸居中，原地转向停止", face_turn_active=False, face_search_side=None)
        else:
            raise ValueError("人脸转向只支持 START_LEFT、START_RIGHT、HEARTBEAT、STOP")
        return self.status_dict()

    def _tuning_values(self) -> dict:
        with self._tuning_lock:
            return {
                "process_fps": self._process_fps, "straight_pwm": self._straight_pwm,
                "correction_deadband": self._fast_config.deadband, "correction_gain": self._fast_config.correction_gain,
                "minimum_correction_pwm": self._fast_config.min_correction_pwm, "maximum_correction_pwm": self._fast_config.max_correction_pwm,
                "green_hue_min": self._fast_config.green_hue_min, "green_hue_max": self._fast_config.green_hue_max,
                "green_saturation_min": self._fast_config.green_saturation_min, "green_dilate_radius_px": self._fast_config.green_dilate_radius_px,
                "green_support_inner_px": self._fast_config.green_support_inner_px, "green_support_outer_px": self._fast_config.green_support_outer_px,
                "green_support_min_ratio": self._fast_config.green_support_min_ratio,
                "turn_90_pwm": self._turn_90.pwm, "turn_90_step_seconds": self._turn_90.step_seconds,
                "turn_180_pwm": self._turn_180.pwm, "turn_180_step_seconds": self._turn_180.step_seconds,
                "face_turn_pwm": FACE_TURN_PWM, "face_turn_pulse_seconds": FACE_TURN_PULSE_SECONDS,
                "face_turn_cooldown_seconds": FACE_TURN_COOLDOWN_SECONDS,
                "face_turn_heartbeat_seconds": FACE_TURN_HEARTBEAT_SECONDS,
                "face_turn_max_seconds": FACE_TURN_MAX_SECONDS,
                "face_turn_line_center_deadband_normalized": FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED,
                "face_turn_line_center_confirm_frames": FACE_TURN_LINE_CENTER_CONFIRM_FRAMES,
                "turn_interstep_pause_seconds": self._turn_interstep_pause_seconds,
                "red_alignment_min_angle": self._red_alignment_min_angle,
                "red_alignment_confirm_frames": self._red_alignment_confirm_frames,
                "turn_90_max_steps": self._turn_90_max_steps, "turn_180_max_steps": self._turn_180_max_steps,
                "return_replay_weight": self._return_replay_weight,
                "return_line_lost_confirm_frames": self._return_line_lost_confirm_frames,
                "return_turn_pause_seconds": self._return_turn_pause_seconds,
                **asdict(self._line_config),
            }

    def _observe_face_turn_line(self, result, frame_width: int) -> bool:
        """Arm after leaving the starting line, then confirm its reacquisition.

        J/L may start while the white stem is already centred.  That initial
        line must not cancel the turn before the first pulse.  Once the line is
        lost or leaves the centre gate, a stable centred reacquisition is a
        valid return-heading stop landmark.
        """
        centre_x = result.center_x if result.valid else None
        centred = bool(
            centre_x is not None
            and frame_width > 0
            and abs((float(centre_x) - frame_width / 2) / (frame_width / 2))
            <= FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED
        )
        if not centred:
            self._face_turn_line_departed = True
            self._face_turn_line_center_streak = 0
            return False
        if not self._face_turn_line_departed:
            return False
        self._face_turn_line_center_streak += 1
        return self._face_turn_line_center_streak >= FACE_TURN_LINE_CENTER_CONFIRM_FRAMES

    def _observe_return_turn_line(self, result, frame_width: int) -> bool:
        centre_x = result.center_x if result.valid else None
        centred = bool(
            centre_x is not None
            and frame_width > 0
            and abs((float(centre_x) - frame_width / 2) / (frame_width / 2))
            <= FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED
        )
        if not centred:
            self._return_line_departed = True
            self._return_line_center_streak = 0
            return False
        if not self._return_line_departed:
            return False
        self._return_line_center_streak += 1
        return self._return_line_center_streak >= FACE_TURN_LINE_CENTER_CONFIRM_FRAMES

    def _load_tuning(self) -> tuple[
        float, int, FastLineConfig, EndLineConfig, float, float, int, int, int,
        float, int, float,
    ]:
        values = {
            "process_fps": 20.0, "straight_pwm": 85,
            "correction_deadband": FastLineConfig().deadband, "correction_gain": FastLineConfig().correction_gain,
            "minimum_correction_pwm": FastLineConfig().min_correction_pwm, "maximum_correction_pwm": FastLineConfig().max_correction_pwm,
            "green_hue_min": FastLineConfig().green_hue_min, "green_hue_max": FastLineConfig().green_hue_max,
            "green_saturation_min": FastLineConfig().green_saturation_min, "green_dilate_radius_px": FastLineConfig().green_dilate_radius_px,
            "green_support_inner_px": FastLineConfig().green_support_inner_px, "green_support_outer_px": FastLineConfig().green_support_outer_px,
            "green_support_min_ratio": FastLineConfig().green_support_min_ratio,
            "turn_interstep_pause_seconds": 2.0,
            "red_alignment_min_angle": 75.0, "red_alignment_confirm_frames": 2,
            "turn_90_max_steps": 4, "turn_180_max_steps": 8,
            "return_replay_weight": .25,
            "return_line_lost_confirm_frames": 3,
            "return_turn_pause_seconds": .20,
            **asdict(EndLineConfig()),
        }
        try:
            stored = self._read_tuning_data()
            if isinstance(stored, dict):
                for key, value in stored.items():
                    if key in TUNING_RULES:
                        kind, minimum, maximum = TUNING_RULES[key]
                        converted = kind(value)
                        if minimum <= converted <= maximum:
                            values[key] = converted
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        process_fps, straight_pwm, fast_config, line_config = self._configs_from_values(values)
        return (process_fps, straight_pwm, fast_config, line_config,
                values["turn_interstep_pause_seconds"], values["red_alignment_min_angle"],
                values["red_alignment_confirm_frames"], values["turn_90_max_steps"],
                values["turn_180_max_steps"], values["return_replay_weight"],
                values["return_line_lost_confirm_frames"],
                values["return_turn_pause_seconds"])

    def _read_tuning_data(self) -> dict:
        if self.config_store is not None:
            return self.config_store.read_section("routes.end_line")
        if not self._tuning_path.exists():
            return {}
        stored = json.loads(self._tuning_path.read_text(encoding="utf-8"))
        return stored if isinstance(stored, dict) else {}

    @staticmethod
    def _configs_from_values(values: dict) -> tuple[float, int, FastLineConfig, EndLineConfig]:
        fast = replace(FastLineConfig(), deadband=values["correction_deadband"], correction_gain=values["correction_gain"], min_correction_pwm=values["minimum_correction_pwm"], max_correction_pwm=values["maximum_correction_pwm"], green_hue_min=values["green_hue_min"], green_hue_max=values["green_hue_max"], green_saturation_min=values["green_saturation_min"], green_dilate_radius_px=values["green_dilate_radius_px"], green_support_inner_px=values["green_support_inner_px"], green_support_outer_px=values["green_support_outer_px"], green_support_min_ratio=values["green_support_min_ratio"])
        line = EndLineConfig(**{key: values[key] for key in asdict(EndLineConfig())})
        return values["process_fps"], values["straight_pwm"], fast, line

    def update_tuning(self, payload: dict) -> dict:
        if not isinstance(payload, dict):
            raise ValueError("调参内容必须是 JSON 对象")
        current = self._tuning_values()
        for key, raw_value in payload.items():
            if key not in TUNING_RULES:
                continue
            kind, minimum, maximum = TUNING_RULES[key]
            try:
                value = kind(raw_value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{key} 不是有效数值") from exc
            if not minimum <= value <= maximum:
                raise ValueError(f"{key} 必须在 {minimum} 到 {maximum} 之间")
            current[key] = value
        if current["minimum_correction_pwm"] > current["maximum_correction_pwm"]:
            raise ValueError("最小修正 PWM 不能大于最大修正 PWM")
        if current["green_hue_min"] > current["green_hue_max"]:
            raise ValueError("绿布色相最小值不能大于最大值")
        if current["green_support_inner_px"] >= current["green_support_outer_px"]:
            raise ValueError("绿布双侧内侧距离必须小于外侧距离")
        process_fps, straight_pwm, fast_config, line_config = self._configs_from_values(current)
        turn_90 = TurnProfile(current["turn_90_pwm"], current["turn_90_step_seconds"])
        turn_180 = TurnProfile(current["turn_180_pwm"], current["turn_180_step_seconds"])
        if self.config_store is not None:
            self.config_store.write_section("routes.end_line", current)
        else:
            self._tuning_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._tuning_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(current, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            temporary.replace(self._tuning_path)
            save_turn_profile(TURN_90_PATH, turn_90)
            save_turn_profile(TURN_180_PATH, turn_180)
        with self._tuning_lock:
            self._process_fps, self._straight_pwm = process_fps, straight_pwm
            self._fast_config, self._line_config = fast_config, line_config
            self._red_detector, self._planner = RedEndBandDetector(line_config), EndLineStopPlanner(line_config)
            self._turn_90, self._turn_180 = turn_90, turn_180
            self._turn_interstep_pause_seconds = current["turn_interstep_pause_seconds"]
            self._red_alignment_min_angle = current["red_alignment_min_angle"]
            self._red_alignment_confirm_frames = current["red_alignment_confirm_frames"]
            self._turn_90_max_steps, self._turn_180_max_steps = current["turn_90_max_steps"], current["turn_180_max_steps"]
            self._return_replay_weight = current["return_replay_weight"]
            self._return_line_lost_confirm_frames = current["return_line_lost_confirm_frames"]
            self._return_turn_pause_seconds = current["return_turn_pause_seconds"]
        return self.status_dict()

    def _open_log(self) -> None:
        directory = SERVICE_ROOT / "runtime_logs" / "end_line"
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._run_log = (directory / f"end_line_turn_{stamp}.jsonl").open("a", encoding="utf-8")

    def _write_log(self, frame: int, result, red, decision, state: str, motor: str, commanded, course_coverage: float) -> None:
        if self._run_log is None:
            return
        controller_status = self.controller.status()
        self._run_log.write(json.dumps({
            "time_utc": datetime.now(timezone.utc).isoformat(), "kind": "end_line_cycle", "frame": frame,
            "white_line": {"valid": result.valid, "center_x": result.center_x, "confidence": result.confidence, "rows": result.centers},
            "green_course_coverage": course_coverage,
            "red_direction_marker": asdict(red), "last_red_side": self._last_red_side, "last_red_seen_frame": self._last_red_seen_frame,
            "planner": {"state": decision.state.value, "reason": decision.reason},
            "gate_enabled": self.gate.enabled(), "state": state, "motor": motor,
            "commanded_pwm": None if commanded is None else {"right": commanded[0], "left": commanded[1]},
            "motor_output": controller_status.get("motor_output"),
        }, ensure_ascii=False) + "\n")
        self._run_log.flush()

    def _run(self) -> None:
        import cv2
        import numpy as np

        last, frame_index = 0.0, 0
        self._open_log()
        self._set_status(running=True, state="ready", detail="纯按键转向：M 解锁后按 Q/E/U/I；未按键时始终停车")
        try:
            while not self._stop.is_set():
                now, jpeg = time.monotonic(), self.camera.latest_jpeg()
                with self._tuning_lock:
                    process_fps = self._process_fps
                if jpeg is None or now - last < 1.0 / process_fps:
                    self._stop.wait(.003)
                    continue
                last = now
                image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                self._flush_return_checkpoints()
                with self._tuning_lock:
                    fast_config, detector, planner, straight_pwm = self._fast_config, self._red_detector, self._planner, self._straight_pwm
                    line_analysis = analyse_fast_line(image, self._last_center_x, fast_config)
                    result = line_analysis.result
                    red = detector.detect(image)
                    decision = planner.step(line_valid=result.valid, red_detected=red.detected)
                if result.center_x is not None:
                    self._last_center_x = result.center_x
                # Red does not cause braking or turning.  It only records the
                # lateral side of its centroid relative to the white stem.
                reference_x = result.center_x if result.center_x is not None else self._last_center_x
                if red.detected and red.center_x is not None and reference_x is not None:
                    self._last_red_side = "LEFT" if red.center_x < reference_x else "RIGHT"
                    self._last_red_seen_frame = frame_index
                state, motor, commanded, detail = decision.state.value, "PAUSED", None, decision.reason
                if self.gate.enabled():
                    recent_red = self._last_red_side is not None and frame_index - self._last_red_seen_frame <= self._line_config.red_direction_memory_frames
                    return_turn_active = self._motion_phase == "RETURN_TURN"
                    return_line_centered = (
                        return_turn_active
                        and self._observe_return_turn_line(result, image.shape[1])
                    )
                    face_turn_active = self._motion_phase == "FACE_CENTER_TURN"
                    line_return_centered = face_turn_active and self._observe_face_turn_line(result, image.shape[1])
                    if return_line_centered:
                        self._stop_motor()
                        self._motion_phase = "RETURN_REPLAY"
                        with self._tuning_lock:
                            self._planner.reset()
                        state, motor, detail = (
                            "RETURN_LINE_REACQUIRED",
                            "STOP_RETURN_HEADING_READY",
                            "掉头后已重新捕获白线，开始视觉主导返程",
                        )
                    elif return_turn_active:
                        if now >= self._return_turn_phase_until:
                            if self._return_turn_pulse_active:
                                self._return_turn_pulse_active = False
                                self._return_turn_phase_until = now + self._return_turn_pause_seconds
                            elif self._return_turn_steps >= self._turn_180_max_steps:
                                self._stop_motor()
                                self._manual_only = True
                                self._return_replay = None
                                self._motion_phase = "MANUAL_COMPLETE"
                                state, motor, detail = (
                                    "RETURN_TURN_TIMEOUT",
                                    "STOP_RETURN_TURN_TIMEOUT",
                                    "返程掉头未重新捕获白线，已安全停车",
                                )
                            else:
                                self._return_turn_steps += 1
                                self._return_turn_pulse_active = True
                                self._return_turn_phase_until = now + self._turn_180.step_seconds
                        if self._motion_phase == "RETURN_TURN" and self._return_turn_pulse_active:
                            commanded = (-self._turn_180.pwm, self._turn_180.pwm)
                            self.controller.set_direct_drive(*commanded)
                            self._motor_active = True
                            state, motor, detail = (
                                f"RETURN_TURN_{self._return_turn_steps}/{self._turn_180_max_steps}",
                                f"RETURN_TURN R={commanded[0]} L={commanded[1]}",
                                "分段原地掉头，等待白线离开后重新居中",
                            )
                        elif self._motion_phase == "RETURN_TURN":
                            self._stop_motor()
                            state, motor = "RETURN_TURN_PAUSE", "STOP_RETURN_TURN_PAUSE"
                    elif face_turn_active and now >= self._face_turn_deadline:
                        self._stop_motor()
                        self._motion_phase, self._face_turn_side = "MANUAL_COMPLETE", None
                        state, motor, detail = "FACE_TURN_HEARTBEAT_TIMEOUT", "STOP_FACE_HEARTBEAT_TIMEOUT", "PC 人脸心跳超时，安全停车"
                        self._set_status(state=state, detail=detail, face_turn_active=False, face_search_side=None)
                    elif line_return_centered:
                        self._stop_motor()
                        self._motion_phase, self._face_turn_side = "MANUAL_COMPLETE", None
                        state, motor, detail = "FACE_TURN_LINE_CENTERED", "STOP_WHITE_LINE_CENTERED", "回转后白线重新居中，连续确认后停车"
                        self._set_status(state=state, detail=detail, face_turn_active=False, face_search_side=None)
                    elif face_turn_active and now - self._face_turn_started >= FACE_TURN_MAX_SECONDS:
                        self._stop_motor()
                        self._motion_phase, self._face_turn_side = "MANUAL_COMPLETE", None
                        state, motor, detail = "FACE_TURN_SEARCH_TIMEOUT", "STOP_FACE_SEARCH_TIMEOUT", f"人脸搜索超过 {FACE_TURN_MAX_SECONDS:.0f}s，安全停车"
                        self._set_status(state=state, detail=detail, face_turn_active=False, face_search_side=None)
                    elif face_turn_active:
                        if now >= self._face_turn_phase_until:
                            self._face_turn_pulse_active = not self._face_turn_pulse_active
                            duration = FACE_TURN_PULSE_SECONDS if self._face_turn_pulse_active else FACE_TURN_COOLDOWN_SECONDS
                            self._face_turn_phase_until = now + duration
                        if self._face_turn_pulse_active:
                            face_pwm = FACE_TURN_PWM
                            commanded = (face_pwm, -face_pwm) if self._face_turn_side == "LEFT" else (-face_pwm, face_pwm)
                            self.controller.set_direct_drive(*commanded); self._motor_active = True
                            state, motor, detail = f"FACE_CENTER_{self._face_turn_side}", f"FACE_CENTER_PULSE R={commanded[0]} L={commanded[1]}", "PC heartbeat active; pulsed pivot until centred"
                        else:
                            self._stop_motor()
                            state, motor, detail = "FACE_CENTER_COOLDOWN", f"STOP_FACE_COOLDOWN_{FACE_TURN_COOLDOWN_SECONDS:.2f}s", "cooldown and capture-stabilisation pause"
                    manual_active = self._motion_phase.startswith("MANUAL")
                    if return_turn_active:
                        pass
                    elif face_turn_active:
                        pass
                    elif manual_active:
                        if red.detected and red.angle_degrees is not None and red.angle_degrees >= self._red_alignment_min_angle:
                            self._red_alignment_streak += 1
                        else:
                            self._red_alignment_streak = 0
                    alignment_confirmed = self._red_alignment_streak >= self._red_alignment_confirm_frames
                    if self._motion_phase == "MANUAL_STEP" and alignment_confirmed:
                        self._stop_motor()
                        self._motion_phase = "MANUAL_COMPLETE"
                        state, motor, detail = "MANUAL_RED_ALIGNED", "STOP_RED_VERTICAL", f"red angle {red.angle_degrees:.1f}° confirmed"
                    elif self._motion_phase == "MANUAL_STEP" and now < self._action_until:
                        profile = self._manual_profile
                        commanded = (profile.pwm, -profile.pwm) if self._pending_turn_side == "LEFT" else (-profile.pwm, profile.pwm)
                        self.controller.set_direct_drive(*commanded); self._motor_active = True
                        state, motor = f"MANUAL_STEP_{self._manual_steps_started}/{self._manual_max_steps}", f"MANUAL_{self._pending_turn_side}_{self._manual_degrees} R={commanded[0]} L={commanded[1]}"
                    elif self._motion_phase == "MANUAL_STEP":
                        self._stop_motor()
                        if self._manual_steps_started < self._manual_max_steps:
                            self._motion_phase = "MANUAL_INTERSTEP_PAUSE"
                            self._action_until = now + self._turn_interstep_pause_seconds
                            state, motor = "MANUAL_INTERSTEP_PAUSE", f"STOP_COOLDOWN_{self._turn_interstep_pause_seconds:.2f}s"
                        else:
                            self._motion_phase = "MANUAL_COMPLETE"
                            state, motor, detail = "MANUAL_ALIGNMENT_TIMEOUT", "STOP_MAX_PULSES", "red line was not vertical before maximum pulses"
                    elif self._motion_phase == "MANUAL_INTERSTEP_PAUSE" and alignment_confirmed:
                        self._stop_motor()
                        self._motion_phase = "MANUAL_COMPLETE"
                        state, motor, detail = "MANUAL_RED_ALIGNED", "STOP_RED_VERTICAL", f"red angle {red.angle_degrees:.1f}° confirmed"
                    elif self._motion_phase == "MANUAL_INTERSTEP_PAUSE" and now < self._action_until:
                        self._stop_motor()
                        state, motor = "MANUAL_INTERSTEP_PAUSE", f"STOP_COOLDOWN_{self._action_until - now:.2f}s"
                    elif self._motion_phase == "MANUAL_INTERSTEP_PAUSE":
                        self._manual_steps_started += 1
                        self._motion_phase = "MANUAL_STEP"
                        self._action_until = now + self._manual_profile.step_seconds
                        state, motor = "MANUAL_NEXT_STEP", "STOP_STARTING_NEXT_STEP"
                    elif self._motion_phase == "MANUAL_COMPLETE":
                        self._stop_motor(); state, motor = "MANUAL_COMPLETE", "STOP_MANUAL_TURN_COMPLETE"
                    elif self._motion_phase == "BRAKE_HOLD" and now < self._action_until:
                        self._stop_motor()
                        state, motor = "BRAKE_BEFORE_90", "STOP_BEFORE_PIVOT"
                    elif self._motion_phase == "BRAKE_HOLD":
                        self._motion_phase, self._action_until = "PIVOT", now + self._turn_90.step_seconds
                    manual_active = self._motion_phase.startswith("MANUAL")
                    if return_turn_active:
                        pass
                    elif face_turn_active:
                        # The continuous face pivot already issued its command
                        # above.  Do not fall through into the manual-only
                        # parking branch and immediately cancel that command.
                        pass
                    elif manual_active:
                        pass
                    elif self._motion_phase == "RETURN_REPLAY":
                        replay_step = (
                            None if self._return_replay is None
                            else self._return_replay.step(now)
                        )
                        if replay_step is None or replay_step.complete:
                            self._stop_motor()
                            self._manual_only = True
                            self._return_replay = None
                            self._motion_phase = "RETURN_COMPLETE"
                            state, motor, detail = (
                                "RETURN_COMPLETE",
                                "STOP_RETURN_COMPLETE",
                                "所有记录路段已逆序完成，返程结束",
                            )
                        else:
                            vision_pwm = pwm_for_line(
                                result, image.shape[1], straight_pwm, fast_config
                            )
                            replay_pwm = replay_step.forward_facing_pwm
                            if vision_pwm is None:
                                self._return_line_lost_frames += 1
                            else:
                                self._return_line_lost_frames = 0
                            if self._return_line_lost_frames >= self._return_line_lost_confirm_frames:
                                self._stop_motor()
                                self._manual_only = True
                                self._return_replay = None
                                self._motion_phase = "MANUAL_COMPLETE"
                                state, motor, detail = (
                                    "RETURN_LINE_LOST",
                                    "STOP_RETURN_LINE_LOST",
                                    "返程连续丢失白线，已安全停车",
                                )
                            elif vision_pwm is None:
                                commanded = tuple(
                                    int(round(value * .5)) for value in replay_pwm
                                )
                                self.controller.set_direct_drive(*commanded)
                                self._motor_active = True
                                state, motor, detail = (
                                    "RETURN_REPLAY_LIMITED",
                                    f"RETURN_LIMITED R={commanded[0]} L={commanded[1]}",
                                    "白线短暂丢失，限速使用记录前馈",
                                )
                            else:
                                commanded = blend_return_pwm(
                                    vision_pwm,
                                    replay_pwm,
                                    self._return_replay_weight,
                                )
                                self.controller.set_direct_drive(*commanded)
                                self._motor_active = True
                                state = "RETURN_VISION_FOLLOW"
                                motor = (
                                    f"RETURN R={commanded[0]} L={commanded[1]} "
                                    f"SEG={replay_step.segment_index}"
                                )
                                detail = (
                                    "摄像头主闭环 + 逆序分段 PWM 前馈"
                                    + ("；已进入上一检查点路段" if replay_step.segment_changed else "")
                                )
                    elif self._manual_only:
                        self._stop_motor()
                        state, motor = "MANUAL_READY", "STOP_WAITING_FOR_Q/E/U/I"
                    elif self._motion_phase == "PIVOT" and now < self._action_until:
                        if self._pending_turn_side == "LEFT":
                            commanded = (self._turn_90.pwm, -self._turn_90.pwm)
                        else:
                            commanded = (-self._turn_90.pwm, self._turn_90.pwm)
                        self.controller.set_direct_drive(*commanded)
                        self._motor_active = True
                        state, motor = f"PIVOT_{self._pending_turn_side}_90", f"PIVOT R={commanded[0]} L={commanded[1]}"
                    elif self._motion_phase == "PIVOT":
                        self._stop_motor()
                        state, motor = "TURN_COMPLETE", "STOP_90_COMPLETE"
                    elif decision.stop:
                        self._stop_motor()
                        self._manual_only = True
                        if recent_red:
                            self._pending_turn_side = self._last_red_side
                            state, motor = "END_REACHED", f"STOP_END_OF_LINE_RED_{self._pending_turn_side}"
                            detail = f"到达端点，红线方向={self._pending_turn_side}，已回到手动模式等待 Q/E 转向"
                        else:
                            state, motor = "END_REACHED_NO_RED", "STOP_END_OF_LINE"
                            detail = "到达端点，无红线方向记录，已回到手动模式"
                    else:
                        precision = red.detected
                        active_fast_config = replace(fast_config, correction_gain=260.0, deadband=.015) if precision else fast_config
                        commanded = pwm_for_line(result, image.shape[1], straight_pwm, active_fast_config)
                        if commanded is None:
                            self._stop_motor()
                            self._manual_only = True
                            state, motor, detail = "LINE_LOST_RETURNED", "STOP_LINE_LOST", "白线丢失，已回到手动模式"
                        else:
                            self.controller.set_direct_drive(*commanded)
                            self._motor_active = True
                            self._return_recorder.record(
                                commanded[0],
                                commanded[1],
                                line_center_x=result.center_x,
                                confidence=result.confidence,
                                wheel_speed=getattr(self.controller, "speed", None),
                                now=now,
                            )
                            motor = f"{'PRECISION' if precision else 'FOLLOW'}_PWM R={commanded[0]} L={commanded[1]}"
                else:
                    self._stop_motor()
                    with self._tuning_lock:
                        self._planner.reset()
                        decision = self._planner.step(line_valid=result.valid, red_detected=red.detected)
                    state, motor, detail = "PAUSED", "STOP_PRESS_M_TO_ARM", "按 M 解锁后，再按 Q/E/U/I 转向"
                overlay = image.copy()
                contours, _hierarchy = cv2.findContours(line_analysis.course_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, (0, 0, 255), 2)
                for y, x, _width in result.centers:
                    cv2.circle(overlay, (int(x), y), 5, (0, 255, 0), -1)
                if red.detected and red.y is not None and red.bottom_y is not None:
                    cv2.line(overlay, (0, red.y), (overlay.shape[1] - 1, red.y), (0, 0, 255), 2)
                    cv2.line(overlay, (0, red.bottom_y), (overlay.shape[1] - 1, red.bottom_y), (0, 80, 255), 1)
                cv2.rectangle(overlay, (10, 10), (1110, 112), (20, 20, 20), cv2.FILLED)
                cv2.putText(overlay, f"END-LINE ADAPTOR: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 0) if self.gate.enabled() else (0, 180, 255), 2)
                cv2.putText(overlay, f"WHITE: valid={result.valid} centre={result.center_x} conf={result.confidence:.2f}  RED: {red.detected} x={red.center_x} angle={red.angle_degrees} side={self._last_red_side}", (18, 66), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
                cv2.putText(overlay, f"STATE: {state}  {decision.reason}  MOTOR: {motor}", (18, 94), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 255, 255), 1)
                ok, encoded = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                if self.gate.enabled() or frame_index % 20 == 0:
                    self._write_log(frame_index, result, red, decision, state, motor, commanded, float(np.mean(line_analysis.course_mask)))
                self._set_status(state=state, detail=detail, frame=frame_index, confidence=result.confidence, line_center_x=result.center_x, green_course_coverage=float(np.mean(line_analysis.course_mask)), green_gate_enabled=fast_config.green_gate_enabled, red_direction_marker=asdict(red), last_red_side=self._last_red_side, last_red_seen_frame=self._last_red_seen_frame, motion_phase=self._motion_phase, motor=motor, face_line_stop_armed=self._face_turn_line_departed, face_line_center_streak=self._face_turn_line_center_streak)
                frame_index += 1
        except Exception as exc:
            import traceback
            traceback.print_exc()
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            if self._run_log is not None:
                self._run_log.close()
            self._set_status(running=False)
