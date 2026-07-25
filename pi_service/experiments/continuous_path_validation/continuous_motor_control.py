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
    # At a clearly visible bend, a loaded four-wheel chassis needs more than
    # the tiny P correction used for ordinary centre-line micro-adjustments.
    sharp_turn_error: float = 0.08
    sharp_turn_correction_pwm: int = 55
    # At a sharp turn, use this signed PWM for the inside wheel. Zero is a
    # controlled stop; a negative value creates a pivot-like turn.
    sharp_turn_inner_pwm: int = 0


def steering_error(observation: PathObservation, config: ContinuousMotorConfig) -> float | None:
    """Combine lateral target and route heading into one signed turn error."""
    target = observation.lookahead_offset
    if target is None:
        return None
    heading = observation.heading or 0.0
    return float(max(-1.0, min(1.0, target + config.heading_weight * heading)))


def path_drive_details(
    observation: PathObservation,
    config: ContinuousMotorConfig,
) -> tuple[tuple[int, int], float | None, int]:
    """Return wheel PWM plus the calculated error and applied correction."""
    base_pwm = max(config.straight_pwm, config.minimum_wheel_pwm)
    error = steering_error(observation, config)
    if error is None:
        return (base_pwm, base_pwm), None, 0
    if abs(error) <= config.correction_deadband:
        return (base_pwm, base_pwm), error, 0
    correction = max(
        config.minimum_correction_pwm,
        min(config.maximum_correction_pwm, math.ceil(abs(error) * config.lookahead_gain)),
    )
    sharp_turn = abs(error) >= config.sharp_turn_error
    if sharp_turn:
        correction = max(correction, config.sharp_turn_correction_pwm)
        correction = min(correction, config.maximum_correction_pwm)
    # Never command a nearly-zero inner wheel during a continuous turn: with
    # this loaded four-wheel chassis that can produce driver current noise but
    # insufficient torque to overcome static friction.
    inner = (
        max(-255, min(255, config.sharp_turn_inner_pwm))
        if sharp_turn
        else max(config.minimum_wheel_pwm, base_pwm - correction)
    )
    outer = min(255, base_pwm + correction)
    # Positive error means the target is right of image centre: slow the right
    # wheel and speed up the left wheel to arc right. The same rule works for
    # any polygon, smooth curve, or line orientation visible ahead.
    return ((inner, outer) if error > 0 else (outer, inner)), error, correction


def path_drive_pwm(observation: PathObservation, config: ContinuousMotorConfig) -> tuple[int, int]:
    """Compatibility wrapper for callers that only need left/right PWM."""
    pair, _error, _correction = path_drive_details(observation, config)
    return pair


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
        self._last_steering_error: float | None = None
        self._last_correction_pwm = 0

    @property
    def steering_diagnostics(self) -> tuple[float | None, int]:
        """Latest P-controller values for JSON logs and field diagnosis."""
        return self._last_steering_error, self._last_correction_pwm

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
            self._last_steering_error, self._last_correction_pwm = None, 0
        else:
            if observation.lookahead_offset is None and self._last_path_pair is not None:
                pair, label = self._last_path_pair, "HOLD_LAST_PATH"
                self._last_steering_error, self._last_correction_pwm = None, 0
            else:
                pair, error, correction = path_drive_details(observation, self.config)
                label = "LOOKAHEAD_P"
                self._last_steering_error, self._last_correction_pwm = error, correction
                self._last_path_pair = pair
        right, left = self.client.send_drive_pwm(*pair)
        self._stopped = False
        if self._last_steering_error is None:
            return f"{label}(R={right},L={left})"
        return f"{label}(e={self._last_steering_error:+.3f},c={self._last_correction_pwm},R={right},L={left})"

    def stop(self) -> None:
        if self._stopped:
            return
        self.client.stop()
        self._forward_active = False
        self._launch_until = 0.0
        self._last_path_pair = None
        self._last_steering_error, self._last_correction_pwm = None, 0
        self._stopped = True
