"""Thin port-5000 bridge for the isolated PC-offload experiment.

The desktop never receives motor authority.  This class owns the only Pi-side
``set_direct_drive`` calls and exposes frame/event methods used by app.py.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import os
import sys
import threading
import time

EXPERIMENT = Path(__file__).resolve().parents[1] / "experiments" / "pc_vision_adaptor_validation"
if str(EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT))
from fast_line import FastLineConfig, find_fast_line, pwm_for_line  # noqa: E402
from protocol import VisionEvent, parse_event  # noqa: E402


@dataclass(frozen=True)
class PcVisionAdaptorConfig:
    process_fps: float = 20.0
    straight_pwm: int = 85
    slow_pwm: int = 55
    pivot_pwm: int = 200
    pivot_seconds: float = 2.5
    remote_event_max_age_ms: int = 750
    remote_armed_timeout_ms: int = 900
    token: str = ""


class PcVisionAdaptorRouteTracker:
    route_mode = "pc_vision_adaptor"

    def __init__(self, controller, camera, publisher, gate, config: PcVisionAdaptorConfig | None = None) -> None:
        self.controller, self.camera, self.publisher, self.gate = controller, camera, publisher, gate
        token = os.environ.get("ROBOT_PC_ADAPTOR_TOKEN", "")
        self.config = config or PcVisionAdaptorConfig(token=token)
        self._stop, self._thread, self._lock = threading.Event(), None, threading.RLock()
        self._frame_seq, self._frame_jpeg, self._frame_at_ms = 0, None, 0
        self._last_center_x, self._last_event_seq, self._event = None, -1, None
        self._event_received_ms, self._pivot_until = 0, 0
        self._motor_active = False
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
            status.update({"enabled": self.gate.enabled(), "pc_event": event, "pc_event_age_ms": max(0, int(time.time()*1000)-self._event_received_ms) if self._event_received_ms else None, "fast_config": asdict(self.config) | {"token": "configured" if self.config.token else "empty"}})
            return status

    def frame_snapshot(self):
        with self._lock:
            return self._frame_jpeg, self._frame_seq, self._frame_at_ms

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

    def _run(self) -> None:
        import cv2
        import numpy as np
        interval, last, frame = 1.0 / max(5.0, self.config.process_fps), 0.0, 0
        self._set_status(running=True, state="ready", detail="Pi 快速跟线已就绪；PC 仅回传视觉事件")
        try:
            while not self._stop.is_set():
                now = time.monotonic(); jpeg = self.camera.latest_jpeg()
                if jpeg is None or now - last < interval:
                    self._stop.wait(.003); continue
                last = now; image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None: continue
                now_ms = int(time.time() * 1000)
                with self._lock:
                    self._frame_seq += 1; self._frame_jpeg = jpeg; self._frame_at_ms = now_ms
                    event, event_age = self._event, now_ms - self._event_received_ms if self._event_received_ms else None
                result = find_fast_line(image, self._last_center_x, FastLineConfig())
                if result.center_x is not None: self._last_center_x = result.center_x
                state, motor = "FAST_FOLLOW", "PAUSED"
                stale_armed = event is not None and event.event != "CLEAR_ARM" and event_age is not None and event_age > self.config.remote_armed_timeout_ms
                if self.gate.enabled():
                    if stale_armed:
                        self._stop_motor(); state, motor = "REMOTE_STALE_STOP", "STOP_REMOTE_EVENT_STALE"
                    elif event and event.event == "BRAKE_NOW":
                        self._stop_motor(); state, motor = "BRAKE", "STOP_PC_BRAKE"
                    elif event and event.event == "PIVOT_REQUEST" and self._pivot_until > now:
                        self.controller.set_direct_drive(-self.config.pivot_pwm, self.config.pivot_pwm); self._motor_active = True; state, motor = "PIVOT", "PIVOT_RIGHT_PC_AUTHORIZED"
                    else:
                        if event and event.event == "PIVOT_REQUEST": self._pivot_until = now + self.config.pivot_seconds
                        pwm = pwm_for_line(result, image.shape[1], self.config.slow_pwm if event and event.event in {"SLOW_DOWN", "TURN_WINDOW_ARMED"} else self.config.straight_pwm)
                        if pwm is None:
                            self._stop_motor(); state, motor = "LINE_LOST_STOP", "STOP_NO_NEAR_LINE"
                        else:
                            self.controller.set_direct_drive(*pwm); self._motor_active = True; motor = f"FAST_PWM R={pwm[0]} L={pwm[1]}"
                else: self._stop_motor(); state = "PAUSED"
                annotated = image.copy()
                for y, x, _w in result.centers: cv2.circle(annotated, (int(x), y), 5, (0, 255, 0), -1)
                cv2.rectangle(annotated, (10, 10), (920, 110), (20,20,20), cv2.FILLED)
                cv2.putText(annotated, f"PC VISION ADAPTOR: {'RUNNING' if self.gate.enabled() else 'PAUSED (press M)'}", (18,38), cv2.FONT_HERSHEY_SIMPLEX,.65,(0,220,0) if self.gate.enabled() else (0,180,255),2)
                cv2.putText(annotated, f"PI FAST: valid={result.valid} centre={result.center_x} conf={result.confidence:.2f}  PC EVENT: {event.event if event else 'none'}", (18,66), cv2.FONT_HERSHEY_SIMPLEX,.46,(255,255,255),1)
                cv2.putText(annotated, f"STATE: {state}  MOTOR: {motor}  PC never sends PWM", (18,94), cv2.FONT_HERSHEY_SIMPLEX,.46,(0,255,255),1)
                ok, encoded = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok: self.publisher.publish(encoded.tobytes())
                self._set_status(state=state, detail=motor, frame=frame, confidence=result.confidence, line_center_x=result.center_x, motor=motor)
                frame += 1
        except Exception as exc: self._set_status(running=False, state="error", detail=str(exc))
        finally: self._stop_motor(); self._set_status(running=False)
