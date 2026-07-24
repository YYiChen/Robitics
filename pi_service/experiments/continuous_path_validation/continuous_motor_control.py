"""Differential P control that follows a generic route lookahead point."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

from pi_service.robot_client import RobotClientConfig, RobotWebClient

from continuous_path_planner import PathIntent


class PathObservation(Protocol):
    lookahead_offset: float | None
    heading: float | None


@dataclass(frozen=True)
class ContinuousMotorConfig:
    controller_url: str
    straight_pwm: int = 75
    launch_pwm: int = 155
    launch_duration_seconds: float = 0.12
    correction_deadband: float = 0.035
    lookahead_gain: float = 160.0
    heading_weight: float = 0.25
    minimum_correction_pwm: int = 20
    maximum_correction_pwm: int = 70


def path_drive_pwm(observation: PathObservation, config: ContinuousMotorConfig) -> tuple[int, int]:
    target = observation.lookahead_offset
    if target is None:
        return config.straight_pwm, config.straight_pwm
    heading = observation.heading or 0.0
    error = float(max(-1.0, min(1.0, target + config.heading_weight * heading)))
    if abs(error) <= config.correction_deadband:
        return config.straight_pwm, config.straight_pwm
    correction = max(
        config.minimum_correction_pwm,
        min(config.maximum_correction_pwm, math.ceil(abs(error) * config.lookahead_gain)),
    )
    inner = max(0, config.straight_pwm - correction)
    outer = min(255, config.straight_pwm + correction)
    # Positive error means the target is right of image centre: slow the right
    # wheel and speed up the left wheel to arc right. The same rule works for
    # any polygon, smooth curve, or line orientation visible ahead.
    return (inner, outer) if error > 0 else (outer, inner)


class ContinuousMotorExecutor:
    def __init__(self, config: ContinuousMotorConfig) -> None:
        self.config = config
        self.client = RobotWebClient(RobotClientConfig(config.controller_url))
        self._forward_active = False
        self._launch_until = 0.0
        self._stopped = False

    def arm(self) -> None:
        self.client.require_arduino_online()
        self.stop()

    def apply(self, intent: PathIntent, observation: PathObservation) -> str:
        if intent is PathIntent.STOP:
            self.stop()
            return "STOP"
        now = time.monotonic()
        if not self._forward_active:
            self._forward_active, self._launch_until = True, now + self.config.launch_duration_seconds
        if now < self._launch_until:
            pair, label = (self.config.launch_pwm, self.config.launch_pwm), "LAUNCH"
        else:
            pair, label = path_drive_pwm(observation, self.config), "LOOKAHEAD_P"
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
