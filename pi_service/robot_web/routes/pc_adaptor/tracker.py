"""Thin port-5000 bridge for the isolated PC-offload experiment.

The desktop never receives motor authority.  This class owns the only Pi-side
``set_direct_drive`` calls and exposes frame/event methods used by app.py.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import os
import threading
import time

from .fast_line import FastLineConfig, find_fast_line, pwm_for_line
from .pi_fast_red import PiFastRedBandPlanner
from .protocol import VisionEvent, parse_event


SERVICE_ROOT = Path(__file__).resolve().parents[3]

# A PC overlay is only a diagnostic view.  Keep its freshness honest: if the
# PC cannot update it for three seconds, show the live Pi fallback instead.
PC_DEBUG_PREVIEW_MAX_AGE_MS = 3_000


@dataclass(frozen=True)
class PcVisionAdaptorConfig:
    process_fps: float = 20.0
    straight_pwm: int = 85
    pivot_pwm: int = 200
    pivot_seconds: float = 2.5
    brake_hold_seconds: float = .18
    # Red is an early-warning signal, not a stop command.  Retain 85 base
    # PWM and tighten only steering sensitivity while it is active.
    precision_gain: float = 260.0
    precision_deadband: float = .015
    reverse_pwm: int = 55
    reverse_seconds: float = .45
    # The PC runs the complete green-field / red-band analysis, which normally
    # takes longer than one Pi video frame.  These are visual-event freshness
    # limits, not PWM authority: Pi still owns PWM and stops when updates end.
    remote_event_max_age_ms: int = 2500
    remote_armed_timeout_ms: int = 3000
    token: str = ""


class PcVisionAdaptorRouteTracker:
    route_mode = "pc_vision_adaptor"

    def __init__(self, controller, camera, publisher, gate, config: PcVisionAdaptorConfig | None = None) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        token = os.environ.get("ROBOT_PC_ADAPTOR_TOKEN", "")
        self.config = config or PcVisionAdaptorConfig(token=token)
        self._stop, self._thread, self._lock = threading.Event(), None, threading.RLock()
        self._frame_seq, self._frame_jpeg, self._frame_at_ms = 0, None, 0
        self._frame_fast_center, self._frame_fast_confidence, self._frame_fast_centers = None, 0.0, ()
        self._pc_preview_jpeg, self._pc_preview_seq, self._pc_preview_at_ms = None, -1, 0
        self._last_center_x, self._last_event_seq, self._event = None, -1, None
        self._event_received_ms, self._action_until = 0, 0.0
        self._last_event_type, self._motion_phase = None, "FOLLOW"
        self._local_red = PiFastRedBandPlanner()
        self._local_red_event, self._local_red_layers = "CLEAR_ARM", ()
        self._effective_event_source = "local"
        self._motor_active = False
        self._run_log = None
        self._status = {"available": True, "running": False, "enabled": False, "mode": self.route_mode, "state": "starting", "detail": "PC adaptor 启动中", "frame": 0, "confidence": 0.0}

    def start(self) -> None:
        if self._thread and self._thread.is_alive(): return
        self._stop.clear(); self._thread = threading.Thread(target=self._run, daemon=True, name="pc-vision-adaptor"); self._thread.start()

    def stop(self) -> None:
        self._stop.set(); self._stop_motor()
        if self._thread and self._thread is not threading.current_thread(): self._thread.join(timeout=1.0)

    def _stop_motor(self) -> None:
        if self._motor_active: self.controller.stop_now()
        self._motor_active = False

    def _open_run_log(self) -> None:
        log_dir = SERVICE_ROOT / "runtime_logs" / "pc_adaptor"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._run_log = (log_dir / f"pc_vision_adaptor_{stamp}.jsonl").open("a", encoding="utf-8")

    @staticmethod
    def _event_priority(event_type: str) -> int:
        """Higher values are physically more urgent and may override PC/local."""
        return {
            "CLEAR_ARM": 0,
            "SLOW_DOWN": 1,
            "TURN_WINDOW_ARMED": 1,
            "BRAKE_NOW": 2,
            "PIVOT_REQUEST": 3,
            "REVERSE_REQUEST": 3,
        }.get(event_type, 0)

    @classmethod
    def _select_effective_event(cls, local_event: str, remote_event: str | None) -> tuple[str, str]:
        """Use Pi-local red evidence first; accept PC evidence only to escalate."""
        if remote_event and cls._event_priority(remote_event) > cls._event_priority(local_event):
            return remote_event, "pc_escalation"
        return local_event, "pi_local_red"

    def _write_run_log(self, *, frame: int, frame_at_ms: int, result, event, local_decision, local_layers, effective_event: str, event_source: str, state: str, motor: str, commanded: tuple[int, int] | None) -> None:
        if self._run_log is None:
            return
        controller_status = self.controller.status() if hasattr(self.controller, "status") else {}
        self._run_log.write(json.dumps({
            "time_utc": datetime.now(timezone.utc).isoformat(), "kind": "pi_control_cycle", "frame": frame, "frame_at_ms": frame_at_ms,
            "confidence": result.confidence, "line_center_x": result.center_x,
            "line_centers": result.centers, "pc_event": event.event if event else None,
            "pc_event_frame_seq": event.frame_seq if event else None,
            "pc_event_captured_at_ms": event.captured_at_ms if event else None,
            "pc_event_age_ms": max(0, frame_at_ms - self._event_received_ms) if self._event_received_ms else None,
            "pi_local_red_event": local_decision.event,
            "pi_local_red_phase": local_decision.phase,
            "pi_local_red_layers": [asdict(layer) for layer in local_layers],
            "effective_event": effective_event, "effective_event_source": event_source,
            "gate_enabled": self.gate.enabled(), "motion_phase": self._motion_phase,
            "state": state, "motor": motor,
            "commanded_pwm": None if commanded is None else {"right": commanded[0], "left": commanded[1]},
            "motor_output": controller_status.get("motor_output"),
        }, ensure_ascii=False) + "\n")
        self._run_log.flush()

    def toggle_drive(self) -> dict:
        enabled = self.gate.toggle()
        if not enabled: self._stop_motor()
        self._set_status(enabled=enabled, detail="PC adaptor 自动行驶已开启" if enabled else "PC adaptor 已暂停，电机已停止")
        return self.status_dict()

    def update_tuning(self, _payload: dict) -> dict:
        raise ValueError("PC adaptor 的 PWM 在 Pi 配置中固定；电脑端只可上报视觉事件")

    def _set_status(self, **changes):
        with self._lock: self._status.update(changes)

    def status_dict(self) -> dict:
        with self._lock:
            status = dict(self._status); event = self._event.event if self._event else None
            status.update({"enabled": self.gate.enabled(), "pc_event": event, "pc_event_age_ms": max(0, int(time.time()*1000)-self._event_received_ms) if self._event_received_ms else None, "pi_local_red_event": self._local_red_event, "pi_local_red_layers": [asdict(layer) for layer in self._local_red_layers], "effective_event_source": self._effective_event_source, "pc_preview_seq": self._pc_preview_seq if self._pc_preview_jpeg else None, "pc_preview_age_ms": max(0, int(time.time()*1000)-self._pc_preview_at_ms) if self._pc_preview_at_ms else None, "fast_config": asdict(self.config) | {"token": "configured" if self.config.token else "empty"}})
            return status

    def frame_snapshot(self):
        with self._lock:
            return self._frame_jpeg, self._frame_seq, self._frame_at_ms, self._frame_fast_center, self._frame_fast_confidence, self._frame_fast_centers

    def submit_remote_event(self, payload: dict) -> dict:
        now_ms = int(time.time() * 1000)
        event = parse_event(payload, token=self.config.token, now_ms=now_ms, max_age_ms=self.config.remote_event_max_age_ms)
        with self._lock:
            if event.frame_seq <= self._last_event_seq:
                raise ValueError("视觉事件帧号未递增")
            if event.frame_seq > self._frame_seq:
                raise ValueError("视觉事件引用了 Pi 尚未发布的帧")
            self._last_event_seq, self._event, self._event_received_ms = event.frame_seq, event, now_ms
        return {"accepted": True, "event": event.event, "frame_seq": event.frame_seq}

    def submit_remote_preview(self, jpeg: bytes, headers) -> dict:
        """Accept only a fresh annotated JPEG for the current Pi frame stream."""
        if not jpeg or len(jpeg) > 3_000_000:
            raise ValueError("PC 标注图片为空或过大")
        if self.config.token and headers.get("X-Vision-Adaptor-Token", "") != self.config.token:
            raise ValueError("PC adaptor token 不匹配")
        try:
            sequence = int(headers["X-Vision-Frame-Seq"])
            captured_at_ms = int(headers["X-Vision-Captured-At-Ms"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("PC 标注必须带帧号和采集时间") from exc
        now_ms = int(time.time() * 1000)
        if now_ms - captured_at_ms > PC_DEBUG_PREVIEW_MAX_AGE_MS:
            raise ValueError("PC 标注帧已过期")
        with self._lock:
            if sequence < self._pc_preview_seq or sequence > self._frame_seq:
                raise ValueError("PC 标注帧号无效或乱序")
            self._pc_preview_jpeg, self._pc_preview_seq, self._pc_preview_at_ms = jpeg, sequence, now_ms
        self.publisher.publish(jpeg)
        return {"accepted": True, "frame_seq": sequence}

    def _run(self) -> None:
        import cv2
        import numpy as np
        interval, last, frame = 1.0 / max(5.0, self.config.process_fps), 0.0, 0
        self._open_run_log()
        self._set_status(running=True, state="ready", detail="Pi 快速跟线已就绪；PC 仅回传视觉事件")
        try:
            while not self._stop.is_set():
                now = time.monotonic(); jpeg = self.camera.latest_jpeg()
                if jpeg is None or now - last < interval:
                    self._stop.wait(.003); continue
                last = now; image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None: continue
                now_ms = int(time.time() * 1000)
                result = find_fast_line(image, self._last_center_x, FastLineConfig())
                local_decision, local_layers = self._local_red.step(image)
                if result.center_x is not None: self._last_center_x = result.center_x
                with self._lock:
                    self._frame_seq += 1; self._frame_jpeg = jpeg; self._frame_at_ms = now_ms
                    self._frame_fast_center, self._frame_fast_confidence, self._frame_fast_centers = result.center_x, result.confidence, result.centers
                    event, event_age = self._event, now_ms - self._event_received_ms if self._event_received_ms else None
                    self._local_red_event, self._local_red_layers = local_decision.event, local_layers
                remote_event_type = event.event if event else None
                event_type, event_source = self._select_effective_event(local_decision.event, remote_event_type)
                self._effective_event_source = event_source
                state, motor, commanded = "FAST_FOLLOW", "PAUSED", None
                # A stale PC message must not cancel a Pi-local red action.
                stale_armed = remote_event_type is not None and remote_event_type != "CLEAR_ARM" and event_age is not None and event_age > self.config.remote_armed_timeout_ms and local_decision.event == "CLEAR_ARM"
                if self.gate.enabled():
                    if stale_armed:
                        self._stop_motor(); state, motor = "REMOTE_STALE_STOP", "STOP_REMOTE_EVENT_STALE"
                    else:
                        # Each remote event may arrive on many fresh frames.
                        # A physical action starts only on a type transition,
                        # never once per JPEG, so repeated PIVOT requests do
                        # not cause endless spinning.
                        if event_type != self._last_event_type:
                            self._last_event_type = event_type
                            if event_type == "BRAKE_NOW":
                                self._motion_phase, self._action_until = "BRAKE_HOLD", now + self.config.brake_hold_seconds
                            elif event_type == "PIVOT_REQUEST":
                                self._motion_phase, self._action_until = "PIVOT", now + self.config.pivot_seconds
                            elif event_type == "REVERSE_REQUEST":
                                self._motion_phase, self._action_until = "REVERSE", now + self.config.reverse_seconds
                            elif event_type == "CLEAR_ARM":
                                self._motion_phase, self._action_until = "FOLLOW", 0.0
                        if self._motion_phase == "BRAKE_HOLD" and now < self._action_until:
                            self._stop_motor(); state, motor = "BRAKE", "STOP_PC_BRAKE_PULSE"
                        elif self._motion_phase == "PIVOT" and now < self._action_until:
                            commanded = (-self.config.pivot_pwm, self.config.pivot_pwm)
                            self.controller.set_direct_drive(*commanded); self._motor_active = True; state, motor = "PIVOT", "PIVOT_RIGHT_PC_AUTHORIZED"
                        elif self._motion_phase == "REVERSE" and now < self._action_until:
                            commanded = (-self.config.reverse_pwm, -self.config.reverse_pwm)
                            self.controller.set_direct_drive(*commanded); self._motor_active = True; state, motor = "REVERSE", "REVERSE_PC_OVERSHOOT_RECOVERY"
                        elif self._motion_phase in {"PIVOT", "REVERSE"}:
                            self._stop_motor(); state, motor = f"{self._motion_phase}_COMPLETE", "STOP_ACTION_COMPLETE"
                        else:
                            # After the short brake pulse, resume the same
                            # base speed.  Red pre-warning tightens steering;
                            # it must never command the 35-PWM static-friction
                            # stall that the old creep mode caused.
                            precision = event_type in {"SLOW_DOWN", "TURN_WINDOW_ARMED", "BRAKE_NOW"} or self._motion_phase == "BRAKE_HOLD"
                            if self._motion_phase == "BRAKE_HOLD": self._motion_phase = "FOLLOW"
                            precision_config = FastLineConfig(correction_gain=self.config.precision_gain, deadband=self.config.precision_deadband)
                            pwm = pwm_for_line(result, image.shape[1], self.config.straight_pwm, precision_config if precision else FastLineConfig())
                            if pwm is None:
                                self._stop_motor(); state, motor = "LINE_LOST_STOP", "STOP_NO_NEAR_LINE"
                            else:
                                commanded = pwm
                                self.controller.set_direct_drive(*commanded); self._motor_active = True; state, motor = ("PRECISION_FOLLOW" if precision else "FAST_FOLLOW"), f"FAST_PWM R={pwm[0]} L={pwm[1]}"
                else: self._stop_motor(); state = "PAUSED"
                annotated = image.copy()
                for y, x, _w in result.centers: cv2.circle(annotated, (int(x), y), 5, (0, 255, 0), -1)
                cv2.rectangle(annotated, (10, 10), (1120, 135), (20,20,20), cv2.FILLED)
                cv2.putText(annotated, f"PC VISION ADAPTOR: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18,38), cv2.FONT_HERSHEY_SIMPLEX,.65,(0,220,0) if self.gate.enabled() else (0,180,255),2)
                cv2.putText(annotated, f"PI FAST: valid={result.valid} centre={result.center_x} conf={result.confidence:.2f}  PC EVENT: {remote_event_type or 'none'}", (18,66), cv2.FONT_HERSHEY_SIMPLEX,.46,(255,255,255),1)
                red_summary = ', '.join(f"y={layer.y}/b={layer.bottom_y}" for layer in local_layers) or "none"
                cv2.putText(annotated, f"PI LOCAL RED: {local_decision.event}/{local_decision.phase} [{red_summary}] -> {event_type} ({event_source})", (18,94), cv2.FONT_HERSHEY_SIMPLEX,.42,(0,80,255),1)
                cv2.putText(annotated, f"STATE: {state}  MOTOR: {motor}  PC never sends PWM", (18,120), cv2.FONT_HERSHEY_SIMPLEX,.46,(0,255,255),1)
                ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    pi_preview = encoded.tobytes()
                    with self._lock:
                        pc_preview = self._pc_preview_jpeg if now_ms - self._pc_preview_at_ms <= PC_DEBUG_PREVIEW_MAX_AGE_MS else None
                    # Fresh PC overlay replaces the light Pi-only fallback.
                    self.publisher.publish(pc_preview or pi_preview)
                # Full evidence is retained for every M-enabled test frame;
                # while merely previewing a paused camera, write one health
                # record per second instead of creating an unnecessary log.
                if self.gate.enabled() or frame % max(1, int(self.config.process_fps)) == 0:
                    self._write_run_log(frame=frame, frame_at_ms=now_ms, result=result, event=event, local_decision=local_decision, local_layers=local_layers, effective_event=event_type, event_source=event_source, state=state, motor=motor, commanded=commanded)
                self._set_status(state=state, detail=motor, frame=frame, confidence=result.confidence, line_center_x=result.center_x, motor=motor, pi_local_red_event=local_decision.event, effective_event=event_type, effective_event_source=event_source)
                frame += 1
        except Exception as exc:
            # This worker owns the control-frame sequence.  Without the full
            # traceback an unexpected vision error only looks like a frozen
            # PC connection, while the real failing source line is lost.
            import traceback

            traceback.print_exc()
            self._set_status(running=False, state="error", detail=str(exc))
        finally:
            self._stop_motor()
            if self._run_log is not None:
                self._run_log.close(); self._run_log = None
            self._set_status(running=False)
