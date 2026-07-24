"""Direct-PWM motor adapter for the isolated straight-line experiment."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

from pi_service.robot_client import RobotClientConfig, RobotWebClient

from line_stop_planner import LineIntent


class OffsetObservation(Protocol):
    offset: float | None


@dataclass(frozen=True)
class StraightMotorConfig:
    controller_url: str
    straight_pwm: int = 65
    launch_pwm: int = 155
    launch_duration_seconds: float = 0.12
    correction_deadband: float = 0.05
    correction_gain: float = 120.0
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 60

    def __post_init__(self) -> None:
        if not 0 <= self.straight_pwm <= 255 or not 0 <= self.launch_pwm <= 255:
            raise ValueError("straight and launch PWM must be in [0, 255]")
        if self.launch_duration_seconds < 0:
            raise ValueError("launch duration must be non-negative")
        if not 0 <= self.correction_deadband <= 1:
            raise ValueError("correction deadband must be in [0, 1]")
        if not 0 <= self.minimum_correction_pwm <= self.maximum_correction_pwm <= 255:
            raise ValueError("correction PWM bounds are invalid")


def drive_pwm_for_offset(observation: OffsetObservation, config: StraightMotorConfig) -> tuple[int, int]:
    """Return (right, left) PWM; the non-zero correction is never below 20."""
    offset = observation.offset
    if offset is None or abs(offset) <= config.correction_deadband:
        return config.straight_pwm, config.straight_pwm

    correction = max(
        config.minimum_correction_pwm,
        min(config.maximum_correction_pwm, math.ceil(abs(float(offset)) * config.correction_gain)),
    )
    inner = max(0, config.straight_pwm - correction)
    outer = min(255, config.straight_pwm + correction)
    # Positive offset: the line is to the right, so the left wheel is faster.
    return (inner, outer) if offset > 0 else (outer, inner)


class StraightMotorExecutor:
    """Heartbeat direct wheel PWM while the line is visible, otherwise STOP."""

    def __init__(self, config: StraightMotorConfig) -> None:
        self.config = config
        self.client = RobotWebClient(RobotClientConfig(config.controller_url))
        self._forward_active = False
        self._launch_until = 0.0
        # Start pessimistically: arming must explicitly send STOP before any
        # camera frame is allowed to command a wheel.
        self._stopped = False

    def arm(self) -> None:
        self.client.require_arduino_online()
        self.stop()

    def apply(self, intent: LineIntent, observation: OffsetObservation) -> str:
        if intent is LineIntent.STOP:
            self.stop()
            return "STOP"

        now = time.monotonic()
        if not self._forward_active:
            self._forward_active = True
            self._launch_until = now + self.config.launch_duration_seconds
        if now < self._launch_until:
            pair = (self.config.launch_pwm, self.config.launch_pwm)
            action = "LAUNCH"
        else:
            pair = drive_pwm_for_offset(observation, self.config)
            action = "P_STRAIGHT"
        right, left = self.client.send_drive_pwm(*pair)
        self._stopped = False
        return f"{action}(R={right},L={left})"

    def stop(self) -> None:
        if self._stopped:
            return
        self.client.stop()
        self._forward_active = False
        self._launch_until = 0.0
        self._stopped = True
