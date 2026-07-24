"""Motor commands for the known fixed clockwise rectangle."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

from pi_service.robot_client import RobotClientConfig, RobotWebClient

from fixed_rectangle_planner import RectangleIntent


class OffsetObservation(Protocol):
    offset: float | None
    valid_bands: int


@dataclass(frozen=True)
class RectangleMotorConfig:
    controller_url: str
    straight_pwm: int = 75
    launch_pwm: int = 155
    launch_duration_seconds: float = 0.12
    pivot_pwm: int = 155
    correction_deadband: float = 0.05
    correction_gain: float = 200.0
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 100


def follow_pwm(observation: OffsetObservation, config: RectangleMotorConfig) -> tuple[int, int]:
    offset = observation.offset
    if offset is None or getattr(observation, "valid_bands", 3) < 3 or abs(offset) <= config.correction_deadband:
        return config.straight_pwm, config.straight_pwm
    correction = max(config.minimum_correction_pwm, min(config.maximum_correction_pwm, math.ceil(abs(float(offset)) * config.correction_gain)))
    inner, outer = max(0, config.straight_pwm - correction), min(255, config.straight_pwm + correction)
    return (inner, outer) if offset > 0 else (outer, inner)


class RectangleMotorExecutor:
    def __init__(self, config: RectangleMotorConfig) -> None:
        self.config = config
        self.client = RobotWebClient(RobotClientConfig(config.controller_url))
        self._forward_active = False
        self._launch_until = 0.0
        self._stopped = False

    def arm(self) -> None:
        self.client.require_arduino_online()
        self.stop()

    def apply(self, intent: RectangleIntent, observation: OffsetObservation) -> str:
        if intent is RectangleIntent.STOP:
            self.stop()
            return "STOP"
        now = time.monotonic()
        if intent is RectangleIntent.PIVOT_RIGHT:
            self._forward_active = False
            pair, label = (-self.config.pivot_pwm, self.config.pivot_pwm), "PIVOT_RIGHT"
        elif intent is RectangleIntent.FOLLOW_LINE:
            if not self._forward_active:
                self._forward_active, self._launch_until = True, now + self.config.launch_duration_seconds
            if now < self._launch_until:
                pair, label = (self.config.launch_pwm, self.config.launch_pwm), "LAUNCH"
            else:
                pair, label = follow_pwm(observation, self.config), "P_FOLLOW"
        else:
            self._forward_active = True
            self._launch_until = 0.0
            pair, label = (self.config.straight_pwm, self.config.straight_pwm), "FIXED_FORWARD"
        right, left = self.client.send_drive_pwm(*pair)
        self._stopped = False
        return f"{label}(R={right},L={left})"

    def stop(self) -> None:
        if self._stopped:
            return
        self.client.stop()
        self._forward_active = False
        self._launch_until = 0.0
        self._stopped = True
