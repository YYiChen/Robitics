"""Small, intentionally non-PWM protocol between the PC and Raspberry Pi.

The PC may report a visual event; it can never choose a wheel PWM.  The Pi
keeps the M-key gate and is the only process that calls ``set_direct_drive``.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any


ALLOWED_EVENTS = frozenset({"SLOW_DOWN", "TURN_WINDOW_ARMED", "BRAKE_NOW", "PIVOT_REQUEST", "REVERSE_REQUEST", "CLEAR_ARM"})


@dataclass(frozen=True)
class VisionEvent:
    event: str
    frame_seq: int
    captured_at_ms: int
    event_at_ms: int


def parse_event(payload: dict[str, Any], *, token: str, now_ms: int | None = None, max_age_ms: int = 750) -> VisionEvent:
    """Validate a PC observation before it can affect the Pi state machine."""
    if not isinstance(payload, dict):
        raise ValueError("事件必须是 JSON 对象")
    if token and payload.get("token") != token:
        raise ValueError("PC adaptor token 不匹配")
    event = str(payload.get("event", "")).upper()
    if event not in ALLOWED_EVENTS:
        raise ValueError("不支持的视觉事件")
    try:
        frame_seq = int(payload["frame_seq"])
        captured_at_ms = int(payload["captured_at_ms"])
        event_at_ms = int(payload.get("event_at_ms", now_ms if now_ms is not None else time.time() * 1000))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("事件必须带 frame_seq 和 captured_at_ms") from exc
    if frame_seq < 0 or captured_at_ms <= 0 or event_at_ms <= 0:
        raise ValueError("事件时间或帧号无效")
    current_ms = int(now_ms if now_ms is not None else time.time() * 1000)
    if current_ms - captured_at_ms > max_age_ms:
        raise ValueError("视觉事件已过期")
    if captured_at_ms - current_ms > 2_000:
        raise ValueError("视觉事件时间来自未来")
    return VisionEvent(event, frame_seq, captured_at_ms, event_at_ms)
