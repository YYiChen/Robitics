"""Port-5000 adaptor with red-line-closed-loop keyboard turning."""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import threading
import time

EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "end_line_turn_validation"
FAST_EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "pc_vision_adaptor_validation"
for path in (EXPERIMENT, FAST_EXPERIMENT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from end_line_logic import EndLineConfig, EndLineStopPlanner, RedEndBandDetector  # noqa: E402
from gated_fast_line import FastLineConfig, analyse_fast_line, pwm_for_line  # noqa: E402
from turn_profiles import TurnProfile, load_turn_profile, save_turn_profile  # noqa: E402


TUNING_PATH = EXPERIMENT / "end_line_web_tuning.json"
TURN_90_PATH = EXPERIMENT / "turn_90.json"
TURN_180_PATH = EXPERIMENT / "turn_180.json"
FACE_TURN_PWM = 90
FACE_PULSE_SECONDS = 0.10
FACE_SETTLE_SECONDS = 0.32
FACE_CENTER_TOLERANCE = 0.10
FACE_CENTER_CONFIRM_FRAMES = 4
FACE_MAX_PULSES = 40
FACE_TURN_TIMEOUT_SECONDS = 25.0
FACE_OBSERVATION_TIMEOUT_SECONDS = 1.0
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
}


class EndLineTurnAdaptorRouteTracker:
    route_mode = "end_line_turn_adaptor"

    def __init__(self, controller, camera, publisher, gate, tuning_path: Path = TUNING_PATH) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        self._stop, self._thread, self._lock, self._tuning_lock = threading.Event(), None, threading.RLock(), threading.RLock()
        self._last_center_x, self._motor_active = None, False
        self._tuning_path = tuning_path
        (self._process_fps, self._straight_pwm, self._fast_config, self._line_config,
         self._turn_interstep_pause_seconds, self._red_alignment_min_angle,
         self._red_alignment_confirm_frames, self._turn_90_max_steps,
         self._turn_180_max_steps) = self._load_tuning()
        self._turn_90 = load_turn_profile(TURN_90_PATH, TurnProfile(200, 1.25), steps=2)
        self._turn_180 = load_turn_profile(TURN_180_PATH, TurnProfile(200, 1.25), steps=4)
        self._planner = EndLineStopPlanner(self._line_config)
        self._red_detector = RedEndBandDetector(self._line_config)
        self._last_red_side, self._last_red_seen_frame = None, -10_000
        self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
        self._manual_degrees, self._manual_profile = None, None
        self._manual_max_steps, self._manual_steps_started = 0, 0
        self._red_alignment_streak = 0
        # This deployment is deliberately keyboard-only: M arms turn commands
        # but must never make the vehicle start following the white line.
        self._manual_only = True
        self._auto_mission_active = False
        self._face_turn_active = False
        self._face_search_side = None
        self._face_offset = None
        self._face_found = False
        self._face_observation_at = 0.0
        self._face_observation_sequence = 0
        self._face_processed_sequence = 0
        self._face_turn_started_at = 0.0
        self._face_center_streak = 0
        self._face_pulse_count = 0
        self._idle_state = "PAUSED"
        self._idle_detail = "按 M 解锁半自动转向，或按 N 启动自动任务"
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
        result.update({
            "enabled": self.gate.enabled(),
            "auto_mission_active": self._auto_mission_active,
            "face_turn_active": self._face_turn_active,
            "face_search_side": self._face_search_side,
            "face_found": self._face_found,
            "face_offset": self._face_offset,
            "face_center_streak": self._face_center_streak,
            "face_pulse_count": self._face_pulse_count,
            "control_mode": (
                f"FACE_{self._face_search_side}"
                if self._face_turn_active
                else "AUTO_LEFT_90" if self._auto_mission_active else "MANUAL_TURN"
            ),
            "tuning": self._tuning_values(),
            "tuning_path": str(self._tuning_path),
        })
        return result

    def _reset_motion(self) -> None:
        self._stop_motor()
        with self._tuning_lock:
            self._planner.reset()
        self._last_red_side, self._last_red_seen_frame = None, -10_000
        self._motion_phase, self._action_until, self._pending_turn_side = "FOLLOW", 0.0, None
        self._manual_degrees, self._manual_profile = None, None
        self._manual_max_steps, self._manual_steps_started, self._red_alignment_streak = 0, 0, 0

    def _clear_face_turn(self) -> None:
        with self._lock:
            self._face_turn_active = False
            self._face_search_side = None
            self._face_offset = None
            self._face_found = False
            self._face_observation_at = 0.0
            self._face_observation_sequence = 0
            self._face_processed_sequence = 0
            self._face_turn_started_at = 0.0
            self._face_center_streak = 0
            self._face_pulse_count = 0

    def toggle_drive(self) -> dict:
        # M remains the semi-automatic diagnostic gate.  During an N mission it
        # doubles as an immediate cancel/stop, but never starts autonomous follow.
        if self._face_turn_active:
            self._clear_face_turn()
            self._manual_only = True
            enabled = self.gate.set_enabled(False)
        elif self._auto_mission_active:
            self._auto_mission_active = False
            self._manual_only = True
            enabled = self.gate.set_enabled(False)
        else:
            self._manual_only = True
            enabled = self.gate.toggle()
        if not enabled:
            self._reset_motion()
            self._idle_state, self._idle_detail = "PAUSED", "已暂停，电机已停止"
        self._set_status(
            enabled=enabled,
            state="MANUAL_READY" if enabled else self._idle_state,
            detail="半自动转向已解锁，等待 Q/E/U/I" if enabled else self._idle_detail,
        )
        return self.status_dict()

    def toggle_auto_left_mission(self) -> dict:
        """N-key mission: follow to the far red terminal, turn left 90°, stop."""
        if self._auto_mission_active:
            self._auto_mission_active = False
            self._manual_only = True
            self.gate.set_enabled(False)
            self._reset_motion()
            self._idle_state, self._idle_detail = "AUTO_CANCELLED", "N 自动任务已取消，电机已停止"
            self._set_status(enabled=False, state=self._idle_state, detail=self._idle_detail)
            return self.status_dict()

        self._clear_face_turn()
        self.gate.set_enabled(False)
        self._reset_motion()
        self._manual_only = False
        self._auto_mission_active = True
        self._pending_turn_side = "LEFT"
        self._idle_state, self._idle_detail = "PAUSED", "按 M 解锁半自动转向，或按 N 启动自动任务"
        self.gate.set_enabled(True)
        self._set_status(
            enabled=True,
            state="AUTO_FOLLOW",
            detail="N 自动任务：沿白线驶向另一端，终点左转 90° 后停车",
        )
        return self.status_dict()

    def request_face_turn(self, direction: str) -> dict:
        """Start J/L face-search turning while PWM authority remains on the Pi."""
        side = str(direction).strip().upper()
        if side not in {"LEFT", "RIGHT"}:
            raise ValueError("人脸转向方向只支持 LEFT 或 RIGHT")
        self.gate.set_enabled(False)
        self._reset_motion()
        self._auto_mission_active = False
        self._manual_only = True
        self._clear_face_turn()
        now = time.monotonic()
        with self._lock:
            self._face_turn_active = True
            self._face_search_side = side
            self._face_turn_started_at = now
            self._motion_phase = "FACE_ALIGN"
        self.gate.set_enabled(True)
        key = "J" if side == "LEFT" else "L"
        self._set_status(
            enabled=True,
            state=f"FACE_SEARCH_{side}",
            detail=f"{key} 人脸转向：等待 DroidCam 观测",
        )
        return self.status_dict()

    def submit_face_observation(self, payload: dict) -> dict:
        if not self._face_turn_active:
            raise ValueError("请先在 DroidCam 窗口按 J 或 L 启动人脸转向")
        if not isinstance(payload.get("found"), bool):
            raise ValueError("found 必须是布尔值")
        found = payload["found"]
        try:
            frame_width = int(payload.get("frame_width"))
        except (TypeError, ValueError):
            raise ValueError("frame_width 必须是正整数") from None
        if frame_width <= 0:
            raise ValueError("frame_width 必须是正整数")
        offset = None
        if found:
            try:
                center_x = float(payload.get("center_x"))
            except (TypeError, ValueError):
                raise ValueError("检测到人脸时 center_x 必须是数值") from None
            if not 0.0 <= center_x <= frame_width:
                raise ValueError("center_x 必须位于画面范围内")
            offset = (center_x - frame_width / 2.0) / max(1.0, frame_width / 2.0)
        with self._lock:
            self._face_found = found
            self._face_offset = offset
            self._face_observation_at = time.monotonic()
            self._face_observation_sequence += 1
        return self.status_dict()

    def cancel_face_turn(self) -> dict:
        if not self._face_turn_active:
            return self.status_dict()
        self.gate.set_enabled(False)
        self._clear_face_turn()
        self._reset_motion()
        self._idle_state, self._idle_detail = "FACE_CANCELLED", "人脸转向已取消，电机已停止"
        self._set_status(enabled=False, state=self._idle_state, detail=self._idle_detail)
        return self.status_dict()

    def _finish_face_turn(self, state: str, detail: str) -> tuple[str, str, None, str]:
        self._stop_motor()
        self.gate.set_enabled(False)
        self._clear_face_turn()
        self._motion_phase = "FOLLOW"
        self._idle_state, self._idle_detail = state, detail
        return state, "STOP_FACE_TURN", None, detail

    def _step_face_turn(self, now: float) -> tuple[str, str, tuple[int, int] | None, str]:
        """Advance one fail-safe pulse/settle/observe cycle."""
        if now - self._face_turn_started_at >= FACE_TURN_TIMEOUT_SECONDS:
            return self._finish_face_turn("FACE_TURN_TIMEOUT", "人脸转向超时，已安全停车")
        observation_age = now - self._face_observation_at if self._face_observation_at else float("inf")
        if observation_age > FACE_OBSERVATION_TIMEOUT_SECONDS:
            self._stop_motor()
            if now - self._face_turn_started_at > FACE_OBSERVATION_TIMEOUT_SECONDS:
                return self._finish_face_turn("FACE_STREAM_LOST", "DroidCam 观测中断，已安全停车")
            return "FACE_WAIT_FRAME", "STOP_WAITING_FOR_DROIDCAM", None, "等待第一帧 DroidCam 人脸观测"

        if self._motion_phase == "FACE_PULSE" and now < self._action_until:
            pwm = FACE_TURN_PWM
            command = (pwm, -pwm) if self._pending_turn_side == "LEFT" else (-pwm, pwm)
            self.controller.set_direct_drive(*command)
            self._motor_active = True
            return f"FACE_PULSE_{self._pending_turn_side}", f"FACE_PULSE R={command[0]} L={command[1]}", command, "人脸短脉冲转向"
        if self._motion_phase == "FACE_PULSE":
            self._stop_motor()
            self._motion_phase = "FACE_SETTLE"
            self._action_until = now + FACE_SETTLE_SECONDS
            return "FACE_SETTLE", "STOP_CAMERA_SETTLE", None, "等待 DroidCam 画面稳定"
        if self._motion_phase == "FACE_SETTLE" and now < self._action_until:
            self._stop_motor()
            return "FACE_SETTLE", "STOP_CAMERA_SETTLE", None, "等待 DroidCam 画面稳定"
        if self._motion_phase == "FACE_SETTLE":
            self._motion_phase = "FACE_ALIGN"

        self._stop_motor()
        if self._face_observation_sequence == self._face_processed_sequence:
            return "FACE_ALIGN", "STOP_WAITING_NEW_FACE_FRAME", None, "等待新的 DroidCam 人脸帧"
        self._face_processed_sequence = self._face_observation_sequence
        offset = self._face_offset
        if offset is not None and abs(offset) <= FACE_CENTER_TOLERANCE:
            self._face_center_streak += 1
            if self._face_center_streak >= FACE_CENTER_CONFIRM_FRAMES:
                return self._finish_face_turn("FACE_CENTERED", "人脸已连续居中确认，转向完成")
            return "FACE_CENTER_CONFIRM", "STOP_FACE_IN_CENTER", None, f"人脸居中确认 {self._face_center_streak}/{FACE_CENTER_CONFIRM_FRAMES}"

        self._face_center_streak = 0
        if self._face_pulse_count >= FACE_MAX_PULSES:
            return self._finish_face_turn("FACE_PULSE_LIMIT", "达到人脸转向最大脉冲数，已安全停车")
        side = self._face_search_side if offset is None else ("LEFT" if offset < 0 else "RIGHT")
        self._pending_turn_side = side
        self._face_pulse_count += 1
        self._motion_phase = "FACE_PULSE"
        self._action_until = now + FACE_PULSE_SECONDS
        return f"FACE_START_{side}", "STOP_BEFORE_FACE_PULSE", None, (
            f"未检测到人脸，继续向{side}搜索"
            if offset is None
            else f"人脸偏差 {offset:+.3f}，向{side}修正"
        )

    def request_manual_turn(self, command: str) -> dict:
        """M-gated turn; profiles are refreshed from disk for every key press."""
        # The JSON files are the source of truth.  Reload here as well as on
        # web save so a profile copied/edited on the Pi is effective on the
        # very next Q/E/U/I press without restarting the web process.
        with self._tuning_lock:
            self._turn_90 = load_turn_profile(TURN_90_PATH, self._turn_90, steps=2)
            self._turn_180 = load_turn_profile(TURN_180_PATH, self._turn_180, steps=4)
            commands = {"LEFT_90": ("LEFT", 90, self._turn_90, self._turn_90_max_steps), "RIGHT_90": ("RIGHT", 90, self._turn_90, self._turn_90_max_steps), "LEFT_180": ("LEFT", 180, self._turn_180, self._turn_180_max_steps), "RIGHT_180": ("RIGHT", 180, self._turn_180, self._turn_180_max_steps)}
        try:
            side, degrees, profile, steps = commands[str(command).upper()]
        except KeyError as exc:
            raise ValueError("手动转向只支持 LEFT_90、RIGHT_90、LEFT_180、RIGHT_180") from exc
        if self._face_turn_active:
            raise ValueError("J/L 人脸转向正在运行；请先停止")
        if self._auto_mission_active:
            raise ValueError("N 自动任务正在运行；请先按 N 或 M 停车")
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
                "turn_interstep_pause_seconds": self._turn_interstep_pause_seconds,
                "red_alignment_min_angle": self._red_alignment_min_angle,
                "red_alignment_confirm_frames": self._red_alignment_confirm_frames,
                "turn_90_max_steps": self._turn_90_max_steps, "turn_180_max_steps": self._turn_180_max_steps,
                **asdict(self._line_config),
            }

    def _load_tuning(self) -> tuple[float, int, FastLineConfig, EndLineConfig, float, float, int, int, int]:
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
            **asdict(EndLineConfig()),
        }
        try:
            stored = json.loads(self._tuning_path.read_text(encoding="utf-8")) if self._tuning_path.exists() else {}
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
                values["turn_180_max_steps"])

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
        self._tuning_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._tuning_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(current, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self._tuning_path)
        turn_90 = TurnProfile(current["turn_90_pwm"], current["turn_90_step_seconds"])
        turn_180 = TurnProfile(current["turn_180_pwm"], current["turn_180_step_seconds"])
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
        return self.status_dict()

    def _open_log(self) -> None:
        directory = EXPERIMENT / "runtime_logs"
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
        self._set_status(running=True, state="ready", detail="M 解锁 Q/E/U/I；N 自动循迹；J/L 使用 DroidCam 人脸居中转向")
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
                    manual_active = self._motion_phase.startswith("MANUAL")
                    face_cycle_active = self._face_turn_active
                    if manual_active:
                        if red.detected and red.angle_degrees is not None and red.angle_degrees >= self._red_alignment_min_angle:
                            self._red_alignment_streak += 1
                        else:
                            self._red_alignment_streak = 0
                    alignment_confirmed = self._red_alignment_streak >= self._red_alignment_confirm_frames
                    if face_cycle_active:
                        state, motor, commanded, detail = self._step_face_turn(now)
                    elif self._motion_phase == "MANUAL_STEP" and alignment_confirmed:
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
                    if face_cycle_active:
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
                        if self._auto_mission_active:
                            self._auto_mission_active = False
                            self._manual_only = True
                            self._motion_phase = "FOLLOW"
                            self.gate.set_enabled(False)
                            self._idle_state, self._idle_detail = "AUTO_COMPLETE", "自动任务完成：已到达另一端并左转 90°"
                            state, motor, detail = self._idle_state, "STOP_AUTO_COMPLETE", self._idle_detail
                        else:
                            state, motor = "TURN_COMPLETE", "STOP_90_COMPLETE"
                    elif decision.stop:
                        self._stop_motor()
                        if recent_red:
                            self._pending_turn_side = "LEFT" if self._auto_mission_active else self._last_red_side
                            self._motion_phase, self._action_until = "BRAKE_HOLD", now + self._line_config.brake_hold_seconds
                            state, motor = "LINE_END_STOP", f"STOP_LINE_LOST_TURN_{self._pending_turn_side}"
                        else:
                            if self._auto_mission_active:
                                self._auto_mission_active = False
                                self._manual_only = True
                                self._motion_phase = "FOLLOW"
                                self.gate.set_enabled(False)
                                self._idle_state, self._idle_detail = "AUTO_SAFE_STOP", "未确认红色终点，自动任务已安全停车"
                                state, motor, detail = self._idle_state, "STOP_LINE_LOST_NO_RED", self._idle_detail
                            else:
                                state, motor = "STOPPED_NO_DIRECTION", "STOP_LINE_LOST_NO_RECENT_RED"
                    else:
                        precision = red.detected
                        active_fast_config = replace(fast_config, correction_gain=260.0, deadband=.015) if precision else fast_config
                        commanded = pwm_for_line(result, image.shape[1], straight_pwm, active_fast_config)
                        if commanded is None:
                            self._stop_motor()
                            state, motor = "STOPPED_UNSAFE_LINE_LOST", "STOP_NO_NEAR_WHITE_LINE"
                        else:
                            self.controller.set_direct_drive(*commanded)
                            self._motor_active = True
                            motor = f"{'PRECISION' if precision else 'FOLLOW'}_PWM R={commanded[0]} L={commanded[1]}"
                else:
                    self._stop_motor()
                    with self._tuning_lock:
                        self._planner.reset()
                        decision = self._planner.step(line_valid=result.valid, red_detected=red.detected)
                    state, motor, detail = self._idle_state, "STOP_WAITING_FOR_M_OR_N", self._idle_detail
                overlay = image.copy()
                contours, _hierarchy = cv2.findContours(line_analysis.course_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours, -1, (0, 0, 255), 2)
                for y, x, _width in result.centers:
                    cv2.circle(overlay, (int(x), y), 5, (0, 255, 0), -1)
                if red.detected and red.y is not None and red.bottom_y is not None:
                    cv2.line(overlay, (0, red.y), (overlay.shape[1] - 1, red.y), (0, 0, 255), 2)
                    cv2.line(overlay, (0, red.bottom_y), (overlay.shape[1] - 1, red.bottom_y), (0, 80, 255), 1)
                cv2.rectangle(overlay, (10, 10), (1110, 112), (20, 20, 20), cv2.FILLED)
                drive_label = (
                    f"FACE {self._face_search_side}"
                    if self._face_turn_active
                    else "AUTO N" if self._auto_mission_active
                    else "MANUAL M" if self.gate.enabled()
                    else "PAUSED (M/N/J/L)"
                )
                cv2.putText(overlay, f"END-LINE ADAPTOR: {drive_label}", (18, 38), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 220, 0) if self.gate.enabled() else (0, 180, 255), 2)
                cv2.putText(overlay, f"WHITE: valid={result.valid} centre={result.center_x} conf={result.confidence:.2f}  RED: {red.detected} x={red.center_x} angle={red.angle_degrees} side={self._last_red_side}", (18, 66), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
                cv2.putText(overlay, f"STATE: {state}  {decision.reason}  MOTOR: {motor}", (18, 94), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 255, 255), 1)
                ok, encoded = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    self.publisher.publish(encoded.tobytes())
                if self.gate.enabled() or frame_index % 20 == 0:
                    self._write_log(frame_index, result, red, decision, state, motor, commanded, float(np.mean(line_analysis.course_mask)))
                self._set_status(state=state, detail=detail, frame=frame_index, confidence=result.confidence, line_center_x=result.center_x, green_course_coverage=float(np.mean(line_analysis.course_mask)), green_gate_enabled=fast_config.green_gate_enabled, red_direction_marker=asdict(red), last_red_side=self._last_red_side, last_red_seen_frame=self._last_red_seen_frame, motion_phase=self._motion_phase, motor=motor)
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
