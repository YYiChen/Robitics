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
    minimum_wheel_pwm: int = 55


def path_drive_pwm(observation: PathObservation, config: ContinuousMotorConfig) -> tuple[int, int]:
    target = observation.lookahead_offset
    base_pwm = max(config.straight_pwm, config.minimum_wheel_pwm)
    if target is None:
        return base_pwm, base_pwm
    heading = observation.heading or 0.0
    error = float(max(-1.0, min(1.0, target + config.heading_weight * heading)))
    if abs(error) <= config.correction_deadband:
        return base_pwm, base_pwm
    correction = max(
        config.minimum_correction_pwm,
        min(config.maximum_correction_pwm, math.ceil(abs(error) * config.lookahead_gain)),
    )
    # Never command a nearly-zero inner wheel during a continuous turn: with
    # this loaded four-wheel chassis that can produce driver current noise but
    # insufficient torque to overcome static friction.
    inner = max(config.minimum_wheel_pwm, base_pwm - correction)
    outer = min(255, base_pwm + correction)
    # Positive error means the target is right of image centre: slow the right
    # wheel and speed up the left wheel to arc right. The same rule works for
    # any polygon, smooth curve, or line orientation visible ahead.
    return (inner, outer) if error > 0 else (outer, inner)


def drive_pwm_with_last_path(
    observation: PathObservation,
    config: ContinuousMotorConfig,
    last_path_pair: tuple[int, int] | None,
) -> tuple[tuple[int, int], str]:
    """Keep the last curvature through a short visual dropout."""
    if observation.lookahead_offset is None and last_path_pair is not None:
        return last_path_pair, "HOLD_LAST_PATH"
    return path_drive_pwm(observation, config), "LOOKAHEAD_P"


class ContinuousMotorExecutor:
    def __init__(self, config: ContinuousMotorConfig) -> None:
        self.config = config
        self.client = RobotWebClient(RobotClientConfig(config.controller_url))
        self._forward_active = False
        self._launch_until = 0.0
        self._stopped = False
        self._last_path_pair: tuple[int, int] | None = None

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
            pair, label = drive_pwm_with_last_path(
                observation,
                self.config,
                self._last_path_pair,
            )
            if observation.lookahead_offset is not None:
                self._last_path_pair = pair
        right, left = self.client.send_drive_pwm(*pair)
        self._stopped = False
        return f"{label}(R={right},L={left})"

    def stop(self) -> None:
        if self._stopped:
            return
        self.client.stop()
        self._forward_active = False
        self._launch_until = 0.0
        self._last_path_pair = None
        self._stopped = True
