"""Port-5000 adaptor with red-line-closed-loop keyboard turning."""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import numpy as np
from pathlib import Path
import threading
import time

from .line_following import FastLineConfig, analyse_fast_line, pwm_for_line
from .perception import EndLineConfig, EndLineStopPlanner, RedEndBandObservation
from .roundtrip import LandmarkTarget, RoundtripPhase, TwoFaceRoundtripPlanner
from .turn_profiles import TurnProfile, load_turn_profile, save_turn_profile


SERVICE_ROOT = Path(__file__).resolve().parents[3]
LEGACY_EXPERIMENT = SERVICE_ROOT / "experiments" / "end_line_turn_validation"
TUNING_PATH = LEGACY_EXPERIMENT / "end_line_web_tuning.json"
TURN_90_PATH = LEGACY_EXPERIMENT / "turn_90.json"
TURN_180_PATH = LEGACY_EXPERIMENT / "turn_180.json"
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
    "face_turn_pwm": (int, 0, 255),
    "face_turn_pulse_seconds": (float, .05, 5.0),
    "face_turn_cooldown_seconds": (float, 0.0, 10.0),
    "face_turn_max_seconds": (float, 1.0, 120.0),
    "face_turn_line_center_deadband_normalized": (float, .01, 1.0),
    "face_turn_line_center_confirm_frames": (int, 1, 30),
}
FACE_TURN_HEARTBEAT_SECONDS = 3.0
# J/L is an operator-triggered, camera-stopped pivot.  It needs enough torque
# to overcome the vehicle's static wheel friction, independently of the short
# duration used by the Q/E preset turn.
FACE_TURN_PWM = 255
FACE_TURN_PULSE_SECONDS = .50
FACE_TURN_COOLDOWN_SECONDS = 2.0
FACE_TURN_MAX_SECONDS = 15.0
FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED = .30
FACE_TURN_LINE_CENTER_CONFIRM_FRAMES = 2
FROZEN_VISUAL_TURN_TUNING = {
    "face_turn_pulse_seconds": FACE_TURN_PULSE_SECONDS,
    "face_turn_line_center_deadband_normalized": FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED,
    "face_turn_line_center_confirm_frames": FACE_TURN_LINE_CENTER_CONFIRM_FRAMES,
}
ROUNDTRIP_LANDMARK_HOLD_SECONDS = 2.0


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
    ) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        self._stop, self._thread, self._lock, self._tuning_lock = threading.Event(), None, threading.RLock(), threading.RLock()
        self._last_center_x, self._motor_active = None, False
        self._tuning_path = tuning_path
        self.config_store = config_store
        (self._process_fps, self._straight_pwm, self._fast_config, self._line_config,
         self._turn_interstep_pause_seconds, self._red_alignment_min_angle,
         self._red_alignment_confirm_frames, self._turn_90_max_steps,
         self._turn_180_max_steps, self._face_turn_pwm,
         self._face_turn_pulse_seconds, self._face_turn_cooldown_seconds,
         self._face_turn_max_seconds, self._face_turn_line_center_deadband_normalized,
         self._face_turn_line_center_confirm_frames) = self._load_tuning()
        if self.config_store is not None:
            stored = self._read_tuning_data()
            self._turn_90 = TurnProfile(int(stored.get("turn_90_pwm", 200)), float(stored.get("turn_90_step_seconds", 1.25)))
            self._turn_180 = TurnProfile(int(stored.get("turn_180_pwm", 200)), float(stored.get("turn_180_step_seconds", 1.25)))
        else:
            self._turn_90 = load_turn_profile(TURN_90_PATH, TurnProfile(200, 1.25), steps=2)
            self._turn_180 = load_turn_profile(TURN_180_PATH, TurnProfile(200, 1.25), steps=4)
        self._planner = EndLineStopPlanner(self._line_config)
        self._last_red_side, self._last_red_seen_frame = None, -10_000
        self._red_alignment_min_angle = 75.0
        self._red_alignment_confirm_frames = 3
        self._red_alignment_streak = 0
        self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
        self._manual_degrees, self._manual_profile = None, None
        self._manual_max_steps, self._manual_steps_started = 0, 0
        # Green mask cache: recompute every N frames, reuse for the rest
        self._cached_green_mask: np.ndarray | None = None
        self._cached_course_mask: np.ndarray | None = None
        self._green_mask_frame_interval: int = 4
        self._face_turn_side, self._face_turn_deadline = None, 0.0
        self._face_turn_started = self._face_turn_phase_until = 0.0
        self._face_turn_pulse_active = False
        self._face_turn_line_departed = False
        self._face_turn_line_center_streak = 0
        self._vision_turn_target = None
        self._roundtrip = TwoFaceRoundtripPlanner()
        self._roundtrip_pending_turn = None
        self._roundtrip_hold_until = 0.0
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
        result.update({"enabled": self.gate.enabled(), "tuning": self._tuning_values(), "tuning_path": tuning_path})
        result["roundtrip"] = self._roundtrip.status_dict()
        return result

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        return self._apply_drive_enabled(enabled)

    def set_drive_enabled(self, enabled: bool) -> dict:
        """Set the M-equivalent motor gate without toggle retry ambiguity."""

        if not isinstance(enabled, bool):
            raise ValueError("enabled 必须是布尔值")
        if self.gate.enabled() == enabled:
            return self.status_dict()
        return self._apply_drive_enabled(self.gate.set_enabled(enabled))

    def _apply_drive_enabled(self, enabled: bool) -> dict:
        if not enabled:
            self._stop_motor()
            with self._tuning_lock:
                self._planner.reset()
            self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
            self._manual_max_steps, self._manual_steps_started = 0, 0
            self._face_turn_side, self._face_turn_deadline = None, 0.0
            self._face_turn_started = self._face_turn_phase_until = 0.0
            self._face_turn_pulse_active = False
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._vision_turn_target = None
            self._roundtrip.reset()
            self._roundtrip_pending_turn = None
            self._roundtrip_hold_until = 0.0
        self._set_status(
            enabled=enabled,
            detail="按键转向已解锁，等待 Q/E/U/I/J/L/H/K" if enabled else "已暂停，电机已停止",
            face_turn_active=False,
            line_turn_active=False,
            vision_turn_target=None,
        )
        return self.status_dict()

    def request_follow_to_end(self) -> dict:
        """N key: follow white line until it ends, then stop and return to manual."""
        if not self.gate.enabled():
            raise ValueError("请先按 M 开启自动电机门控，再触发 N 巡线")
        if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE"}:
            raise ValueError("当前已有转向动作，请等待其完成或按 M 停止")
        self._manual_only = False
        self._planner.reset()
        self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
        self._set_status(state="FOLLOWING_TO_END", detail="N 已触发，沿白线行驶至尽头")
        return self.status_dict()

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
            if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE", "FACE_CENTER_TURN", "ROUNDTRIP_HOLD"}:
                raise ValueError("当前已有其他转向动作，请等待完成或按 M 停止")
            self._manual_only = True
            self._stop_motor()
            now = time.monotonic()
            self._face_turn_side = "LEFT" if command.endswith("LEFT") else "RIGHT"
            self._face_turn_deadline = now + FACE_TURN_HEARTBEAT_SECONDS
            self._face_turn_started = now
            self._face_turn_phase_until = now + self._face_turn_pulse_seconds
            self._face_turn_pulse_active = True
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._vision_turn_target = "FACE"
            self._motion_phase = "FACE_CENTER_TURN"
            self._set_status(
                state="FACE_CENTER_TURN",
                detail=f"PC 人脸居中：脉冲原地{self._face_turn_side}转，等待心跳",
                face_turn_active=True,
                line_turn_active=False,
                face_search_side=self._face_turn_side,
                vision_turn_target=self._vision_turn_target,
            )
        elif command == "HEARTBEAT":
            if self._motion_phase != "FACE_CENTER_TURN" or self._face_turn_side is None:
                raise ValueError("当前没有进行中的人脸居中转向")
            self._face_turn_deadline = time.monotonic() + FACE_TURN_HEARTBEAT_SECONDS
        elif command == "STOP":
            if self._motion_phase != "FACE_CENTER_TURN":
                return self.status_dict()
            completes_roundtrip = (
                self._roundtrip.expected_turn() is not None
                and self._roundtrip.expected_turn().target == LandmarkTarget.FACE
            )
            self._stop_motor()
            self._motion_phase, self._face_turn_side, self._face_turn_deadline = "MANUAL_COMPLETE", None, 0.0
            self._face_turn_started = self._face_turn_phase_until = 0.0
            self._face_turn_pulse_active = False
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._vision_turn_target = None
            self._set_status(
                state="FACE_CENTERED_STOP",
                detail="PC 已确认人脸居中，原地转向停止",
                face_turn_active=False,
                line_turn_active=False,
                face_search_side=None,
                vision_turn_target=None,
            )
            if completes_roundtrip:
                self._complete_roundtrip_target(LandmarkTarget.FACE)
        else:
            raise ValueError("人脸转向只支持 START_LEFT、START_RIGHT、HEARTBEAT、STOP")
        return self.status_dict()

    def request_roundtrip_start(self, sweep_side: str) -> dict:
        """Follow outbound, visit two faces, and finish aligned for return."""
        if not self.gate.enabled():
            raise ValueError("请先按 M 开启自动电机门控，再启动双人脸序列")
        if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE"}:
            raise ValueError("当前已有动作，请先停止或等待完成")
        self._roundtrip.start(sweep_side)
        self._manual_only = False
        self._planner.reset()
        self._last_red_side, self._last_red_seen_frame = None, -10_000
        self._roundtrip_pending_turn = None
        self._roundtrip_hold_until = 0.0
        self._motion_phase = "FOLLOW"
        self._set_status(
            state="ROUNDTRIP_FOLLOW_OUTBOUND",
            detail=f"双人脸序列已启动：先巡线到端点，再向 {self._roundtrip.sweep_side} 扫描",
            roundtrip=self._roundtrip.status_dict(),
        )
        return self.status_dict()

    def request_roundtrip_return(self) -> dict:
        if not self.gate.enabled():
            raise ValueError("请先按 M 开启自动电机门控，再启动返程")
        self._roundtrip.start_return()
        self._manual_only = False
        self._planner.reset()
        self._motion_phase = "FOLLOW"
        self._set_status(
            state="ROUNDTRIP_FOLLOW_RETURN",
            detail="白线已对准，开始返程巡线；到另一端自动停车",
            roundtrip=self._roundtrip.status_dict(),
        )
        return self.status_dict()

    def request_roundtrip_stop(self) -> dict:
        self._stop_motor()
        self._roundtrip.reset()
        self._roundtrip_pending_turn = None
        self._roundtrip_hold_until = 0.0
        self._vision_turn_target = None
        self._motion_phase = "MANUAL_COMPLETE"
        self._face_turn_side, self._face_turn_deadline = None, 0.0
        self._manual_only = True
        self._set_status(
            state="ROUNDTRIP_STOPPED",
            detail="双人脸序列已停止",
            face_turn_active=False,
            line_turn_active=False,
            vision_turn_target=None,
            roundtrip=self._roundtrip.status_dict(),
        )
        return self.status_dict()

    def _schedule_roundtrip_turn(self, now: float | None = None) -> None:
        instruction = self._roundtrip.expected_turn()
        if instruction is None:
            raise ValueError(f"{self._roundtrip.phase.value} 没有待执行转向")
        current = time.monotonic() if now is None else now
        self._stop_motor()
        self._roundtrip_pending_turn = instruction
        self._roundtrip_hold_until = current + ROUNDTRIP_LANDMARK_HOLD_SECONDS
        self._motion_phase = "ROUNDTRIP_HOLD"
        self._manual_only = True
        self._set_status(
            state="ROUNDTRIP_HOLD",
            detail=f"停车稳定 {ROUNDTRIP_LANDMARK_HOLD_SECONDS:.1f}s，随后 {instruction.side} 转向寻找 {instruction.target.value}",
            roundtrip=self._roundtrip.status_dict(),
        )

    def _complete_roundtrip_target(self, target: LandmarkTarget) -> None:
        self._roundtrip.target_reached(target)
        if self._roundtrip.phase == RoundtripPhase.READY_RETURN:
            self._stop_motor()
            self._motion_phase = "MANUAL_COMPLETE"
            self._manual_only = True
            self._roundtrip_pending_turn = None
            self._set_status(
                state="ROUNDTRIP_READY_RETURN",
                detail="人脸 1、白线中位、人脸 2、返程白线均已完成；等待网页“开始返程”",
                roundtrip=self._roundtrip.status_dict(),
            )
        else:
            self._schedule_roundtrip_turn()

    def request_line_center_turn(self, command: str) -> dict:
        """H/K: pivot until the starting line is left and then reacquired.

        Unlike J/L, this action is closed locally by the Pi white-line
        detector and therefore does not require a PC face-detection heartbeat.
        The initial centred line is deliberately ignored so the first frame
        cannot cancel the requested turn.
        """
        command = str(command).upper()
        if command in {"START_LEFT", "START_RIGHT"}:
            if not self.gate.enabled():
                raise ValueError("请先按 M 开启自动电机门控，再启动白线居中转向")
            if self._motion_phase not in {"FOLLOW", "MANUAL_COMPLETE", "TURN_COMPLETE", "LINE_CENTER_TURN", "ROUNDTRIP_HOLD"}:
                raise ValueError("当前已有其他转向动作，请等待完成或按 M 停止")
            self._manual_only = True
            self._stop_motor()
            now = time.monotonic()
            self._face_turn_side = "LEFT" if command.endswith("LEFT") else "RIGHT"
            self._face_turn_deadline = 0.0
            self._face_turn_started = now
            self._face_turn_phase_until = now + self._face_turn_pulse_seconds
            self._face_turn_pulse_active = True
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._vision_turn_target = "WHITE_LINE"
            self._motion_phase = "LINE_CENTER_TURN"
            self._set_status(
                state="LINE_CENTER_TURN",
                detail=f"白线居中：脉冲原地{self._face_turn_side}转，等待白线离开后重新居中",
                face_turn_active=False,
                line_turn_active=True,
                face_search_side=self._face_turn_side,
                vision_turn_target=self._vision_turn_target,
            )
        elif command == "STOP":
            if self._motion_phase != "LINE_CENTER_TURN":
                return self.status_dict()
            self._stop_motor()
            self._motion_phase, self._face_turn_side, self._face_turn_deadline = "MANUAL_COMPLETE", None, 0.0
            self._face_turn_started = self._face_turn_phase_until = 0.0
            self._face_turn_pulse_active = False
            self._face_turn_line_departed = False
            self._face_turn_line_center_streak = 0
            self._vision_turn_target = None
            self._set_status(
                state="LINE_TURN_STOPPED",
                detail="白线居中转向已停止",
                line_turn_active=False,
                face_search_side=None,
                vision_turn_target=None,
            )
        else:
            raise ValueError("白线转向只支持 START_LEFT、START_RIGHT、STOP")
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
                "face_turn_pwm": self._face_turn_pwm, "face_turn_pulse_seconds": self._face_turn_pulse_seconds,
                "face_turn_cooldown_seconds": self._face_turn_cooldown_seconds,
                "face_turn_heartbeat_seconds": FACE_TURN_HEARTBEAT_SECONDS,
                "face_turn_max_seconds": self._face_turn_max_seconds,
                "face_turn_line_center_deadband_normalized": self._face_turn_line_center_deadband_normalized,
                "face_turn_line_center_confirm_frames": self._face_turn_line_center_confirm_frames,
                "turn_interstep_pause_seconds": self._turn_interstep_pause_seconds,
                "red_alignment_min_angle": self._red_alignment_min_angle,
                "red_alignment_confirm_frames": self._red_alignment_confirm_frames,
                "turn_90_max_steps": self._turn_90_max_steps, "turn_180_max_steps": self._turn_180_max_steps,
                **asdict(self._line_config),
            }

    def _observe_line_turn_reacquisition(self, result, frame_width: int) -> bool:
        """Arm after leaving the starting line, then confirm its reacquisition.

        H/K may start while the white stem is already centred.  That initial
        line must not cancel the turn before the first pulse.  Once the line is
        lost or leaves the centre gate, a stable centred reacquisition is a
        valid return-heading stop landmark.
        """
        centre_x = result.center_x if result.valid else None
        centred = bool(
            centre_x is not None
            and frame_width > 0
            and abs((float(centre_x) - frame_width / 2) / (frame_width / 2))
            <= self._face_turn_line_center_deadband_normalized
        )
        if not centred:
            self._face_turn_line_departed = True
            self._face_turn_line_center_streak = 0
            return False
        if not self._face_turn_line_departed:
            return False
        self._face_turn_line_center_streak += 1
        return self._face_turn_line_center_streak >= self._face_turn_line_center_confirm_frames

    def _load_tuning(self) -> tuple:
        values = {
            "process_fps": 10.0, "straight_pwm": 85,
            "correction_deadband": FastLineConfig().deadband, "correction_gain": FastLineConfig().correction_gain,
            "minimum_correction_pwm": FastLineConfig().min_correction_pwm, "maximum_correction_pwm": FastLineConfig().max_correction_pwm,
            "green_hue_min": FastLineConfig().green_hue_min, "green_hue_max": FastLineConfig().green_hue_max,
            "green_saturation_min": FastLineConfig().green_saturation_min, "green_dilate_radius_px": FastLineConfig().green_dilate_radius_px,
            "green_support_inner_px": FastLineConfig().green_support_inner_px, "green_support_outer_px": FastLineConfig().green_support_outer_px,
            "green_support_min_ratio": FastLineConfig().green_support_min_ratio,
            "turn_interstep_pause_seconds": 2.0,
            "red_alignment_min_angle": 75.0, "red_alignment_confirm_frames": 2,
            "turn_90_max_steps": 4, "turn_180_max_steps": 8,
            "face_turn_pwm": FACE_TURN_PWM,
            "face_turn_pulse_seconds": FACE_TURN_PULSE_SECONDS,
            "face_turn_cooldown_seconds": FACE_TURN_COOLDOWN_SECONDS,
            "face_turn_max_seconds": FACE_TURN_MAX_SECONDS,
            "face_turn_line_center_deadband_normalized": FACE_TURN_LINE_CENTER_DEADBAND_NORMALIZED,
            "face_turn_line_center_confirm_frames": FACE_TURN_LINE_CENTER_CONFIRM_FRAMES,
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
        # These three timing/white-line gates are a validated vehicle preset.
        # Persisted files and API payloads must not override them.
        values.update(FROZEN_VISUAL_TURN_TUNING)
        process_fps, straight_pwm, fast_config, line_config = self._configs_from_values(values)
        return (process_fps, straight_pwm, fast_config, line_config,
                values["turn_interstep_pause_seconds"], values["red_alignment_min_angle"],
                values["red_alignment_confirm_frames"], values["turn_90_max_steps"],
                values["turn_180_max_steps"], values["face_turn_pwm"],
                values["face_turn_pulse_seconds"], values["face_turn_cooldown_seconds"],
                values["face_turn_max_seconds"],
                values["face_turn_line_center_deadband_normalized"],
                values["face_turn_line_center_confirm_frames"])

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
        current.update(FROZEN_VISUAL_TURN_TUNING)
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
            self._planner = EndLineStopPlanner(line_config)
            self._turn_90, self._turn_180 = turn_90, turn_180
            self._turn_interstep_pause_seconds = current["turn_interstep_pause_seconds"]
            self._red_alignment_min_angle = current["red_alignment_min_angle"]
            self._red_alignment_confirm_frames = current["red_alignment_confirm_frames"]
            self._turn_90_max_steps, self._turn_180_max_steps = current["turn_90_max_steps"], current["turn_180_max_steps"]
            self._face_turn_pwm = current["face_turn_pwm"]
            self._face_turn_pulse_seconds = current["face_turn_pulse_seconds"]
            self._face_turn_cooldown_seconds = current["face_turn_cooldown_seconds"]
            self._face_turn_max_seconds = current["face_turn_max_seconds"]
            self._face_turn_line_center_deadband_normalized = current["face_turn_line_center_deadband_normalized"]
            self._face_turn_line_center_confirm_frames = current["face_turn_line_center_confirm_frames"]
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
                with self._tuning_lock:
                    fast_config, planner, straight_pwm = self._fast_config, self._planner, self._straight_pwm
                    # Green mask caching: recompute only every N frames
                    use_cache = (self._cached_green_mask is not None
                                 and self._cached_course_mask is not None
                                 and frame_index % self._green_mask_frame_interval != 0)
                    line_analysis = analyse_fast_line(
                        image, self._last_center_x, fast_config,
                        cached_course_mask=self._cached_course_mask if use_cache else None,
                    )
                    if not use_cache:
                        self._cached_green_mask = line_analysis.green_mask.copy()
                        self._cached_course_mask = line_analysis.course_mask.copy()
                    result = line_analysis.result
                    red = RedEndBandObservation(False)
                    decision = planner.step(line_valid=result.valid)
                if result.center_x is not None:
                    self._last_center_x = result.center_x
                state, motor, commanded, detail = decision.state.value, "PAUSED", None, decision.reason
                if self.gate.enabled():
                    roundtrip_hold_active = self._motion_phase == "ROUNDTRIP_HOLD"
                    if roundtrip_hold_active and now >= self._roundtrip_hold_until:
                        instruction = self._roundtrip_pending_turn
                        if instruction is None:
                            self.request_roundtrip_stop()
                        else:
                            self._roundtrip_pending_turn = None
                            command = f"START_{instruction.side}"
                            if instruction.target == LandmarkTarget.FACE:
                                self.request_face_center_turn(command)
                            else:
                                self.request_line_center_turn(command)
                        roundtrip_hold_active = self._motion_phase == "ROUNDTRIP_HOLD"
                    face_turn_active = self._motion_phase == "FACE_CENTER_TURN"
                    line_turn_active = self._motion_phase == "LINE_CENTER_TURN"
                    vision_turn_active = face_turn_active or line_turn_active
                    route_action_active = vision_turn_active or roundtrip_hold_active
                    line_return_centered = line_turn_active and self._observe_line_turn_reacquisition(result, image.shape[1])
                    if roundtrip_hold_active:
                        self._stop_motor()
                        state = "ROUNDTRIP_HOLD"
                        motor = f"STOP_ROUNDTRIP_HOLD_{max(0.0, self._roundtrip_hold_until - now):.2f}s"
                        detail = "停车稳定后再开始下一段视觉转向"
                    elif face_turn_active and now >= self._face_turn_deadline:
                        self._stop_motor()
                        self._motion_phase, self._face_turn_side = "MANUAL_COMPLETE", None
                        self._vision_turn_target = None
                        state, motor, detail = "FACE_TURN_HEARTBEAT_TIMEOUT", "STOP_FACE_HEARTBEAT_TIMEOUT", "PC 人脸心跳超时，安全停车"
                        self._set_status(state=state, detail=detail, face_turn_active=False, line_turn_active=False, face_search_side=None, vision_turn_target=None)
                    elif line_return_centered:
                        completes_roundtrip = (
                            self._roundtrip.expected_turn() is not None
                            and self._roundtrip.expected_turn().target == LandmarkTarget.WHITE_LINE
                        )
                        self._stop_motor()
                        self._motion_phase, self._face_turn_side = "MANUAL_COMPLETE", None
                        self._vision_turn_target = None
                        state, motor, detail = "LINE_TURN_CENTERED", "STOP_WHITE_LINE_CENTERED", "白线离开后重新居中，连续确认后停车"
                        self._set_status(state=state, detail=detail, face_turn_active=False, line_turn_active=False, face_search_side=None, vision_turn_target=None)
                        if completes_roundtrip:
                            self._complete_roundtrip_target(LandmarkTarget.WHITE_LINE)
                    elif vision_turn_active and now - self._face_turn_started >= self._face_turn_max_seconds:
                        self._stop_motor()
                        self._motion_phase, self._face_turn_side = "MANUAL_COMPLETE", None
                        timed_out_target = self._vision_turn_target
                        self._vision_turn_target = None
                        state = "FACE_TURN_SEARCH_TIMEOUT" if timed_out_target == "FACE" else "LINE_TURN_SEARCH_TIMEOUT"
                        motor = "STOP_FACE_SEARCH_TIMEOUT" if timed_out_target == "FACE" else "STOP_LINE_SEARCH_TIMEOUT"
                        target_text = "人脸" if timed_out_target == "FACE" else "白线"
                        detail = f"{target_text}搜索超过 {self._face_turn_max_seconds:.0f}s，安全停车"
                        self._set_status(state=state, detail=detail, face_turn_active=False, line_turn_active=False, face_search_side=None, vision_turn_target=None)
                    elif vision_turn_active:
                        if now >= self._face_turn_phase_until:
                            self._face_turn_pulse_active = not self._face_turn_pulse_active
                            duration = self._face_turn_pulse_seconds if self._face_turn_pulse_active else self._face_turn_cooldown_seconds
                            self._face_turn_phase_until = now + duration
                        if self._face_turn_pulse_active:
                            face_pwm = self._face_turn_pwm
                            commanded = (face_pwm, -face_pwm) if self._face_turn_side == "LEFT" else (-face_pwm, face_pwm)
                            self.controller.set_direct_drive(*commanded); self._motor_active = True
                            target_name = "FACE" if face_turn_active else "LINE"
                            state, motor = f"{target_name}_CENTER_{self._face_turn_side}", f"{target_name}_CENTER_PULSE R={commanded[0]} L={commanded[1]}"
                            detail = "PC heartbeat active; pulsed pivot until face centred" if face_turn_active else "Pi white-line closed loop; pulsed pivot until line reacquired"
                        else:
                            self._stop_motor()
                            target_name = "FACE" if face_turn_active else "LINE"
                            state, motor, detail = f"{target_name}_CENTER_COOLDOWN", f"STOP_{target_name}_COOLDOWN_{self._face_turn_cooldown_seconds:.2f}s", "cooldown and capture-stabilisation pause"
                    manual_active = self._motion_phase.startswith("MANUAL")
                    if route_action_active:
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
                    if route_action_active:
                        # The continuous vision pivot already issued its command
                        # above.  Do not fall through into the manual-only
                        # parking branch and immediately cancel that command.
                        pass
                    elif manual_active:
                        pass
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
                        state, motor = "END_REACHED", "STOP_END_OF_LINE"
                        detail = "到达白线端点，已回到手动模式"
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
                            motor = f"{'PRECISION' if precision else 'FOLLOW'}_PWM R={commanded[0]} L={commanded[1]}"
                else:
                    self._stop_motor()
                    with self._tuning_lock:
                        self._planner.reset()
                        decision = self._planner.step(line_valid=result.valid)
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
                self._set_status(state=state, detail=detail, frame=frame_index, confidence=result.confidence, line_center_x=result.center_x, green_course_coverage=float(np.mean(line_analysis.course_mask)), green_gate_enabled=fast_config.green_gate_enabled, red_direction_marker=asdict(red), last_red_side=self._last_red_side, last_red_seen_frame=self._last_red_seen_frame, motion_phase=self._motion_phase, motor=motor, face_line_stop_armed=self._face_turn_line_departed, face_line_center_streak=self._face_turn_line_center_streak, vision_turn_target=self._vision_turn_target, line_turn_active=self._motion_phase == "LINE_CENTER_TURN")
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
