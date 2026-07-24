"""HTTP motor executor for the existing Pi robot-web controller.

This adapter is deliberately separate from vision. It only becomes active when
the live monitor receives ``--enable-motors``.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Protocol

from rectangle_route_planner import PlannerDecision, RouteIntent, RouteState
from pi_service.robot_client import RobotClientConfig, RobotWebClient


class LineObservation(Protocol):
    offset: float | None


@dataclass(frozen=True)
class MotorControlConfig:
    controller_url: str
    straight_pwm: int = 110
    pivot_pwm: int = 155
    curve_outer_pwm: int = 180
    curve_inner_pwm: int = 60
    correction_deadband: float = 0.05
    p_gain: float = 200.0
    pwm_max_step: int = 12
    command_interval_seconds: float = 0.05


def action_for_decision(
    decision: PlannerDecision,
    observation: LineObservation,
    config: MotorControlConfig,
) -> str:
    """Map safe route intent to the Pi controller's existing action names."""
    if decision.intent is RouteIntent.STOP:
        return "STOP"
    if decision.intent is RouteIntent.TURN_RIGHT:
        return "PR"
    if decision.state is RouteState.APPROACHING_RIGHT_CORNER:
        # The planner deliberately bridges a short camera-only blind zone
        # before the physical corner. Keep travelling forward in that zone.
        return "F"
    if observation.offset is None:
        return "STOP"
    if observation.offset < -config.correction_deadband:
        return "FL"
    if observation.offset > config.correction_deadband:
        return "FR"
    return "F"


def proportional_drive_pwm(
    observation: LineObservation,
    config: MotorControlConfig,
) -> tuple[int, int] | None:
    """Map normalized image offset to right/left PWM with P control only."""
    if observation.offset is None:
        return None
    error = float(observation.offset)
    if abs(error) <= config.correction_deadband:
        return config.straight_pwm, config.straight_pwm

    # Positive offset means the line is right of centre: left must be faster.
    correction = abs(error) * config.p_gain
    outer = min(config.curve_outer_pwm, round(config.straight_pwm + correction))
    inner = max(config.curve_inner_pwm, round(config.straight_pwm - correction))
    return (inner, outer) if error > 0 else (outer, inner)


class RobotWebMotorExecutor:
    """Configure and heartbeat the Pi ``/api/action`` control endpoint."""

    def __init__(self, config: MotorControlConfig) -> None:
        self.config = config
        self.client = RobotWebClient(RobotClientConfig(config.controller_url))
        self._last_action: str | tuple[int, int] | None = None
        self._last_sent_at = 0.0
        self._last_drive_pwm: tuple[int, int] | None = None
        self._direct_drive_supported = True

    def configure(self) -> None:
        """Require an online Arduino, then apply the requested PWM profiles."""
        status = self.client.require_arduino_online()
        robot = status.get("robot", {})
        current = robot.get("config", {})
        profiles = dict(current.get("profiles", {}))
        straight = self.config.straight_pwm
        outer = self.config.curve_outer_pwm
        inner = self.config.curve_inner_pwm
        pivot = self.config.pivot_pwm
        profiles.update(
            {
                "F": {"rf": straight, "lf": straight, "lr": straight, "rr": straight},
                # FL: right side is the outside of a left correction; FR is
                # the mirror image.  Keep correction power independent from
                # normal straight-line power.
                "FL": {"rf": outer, "lf": inner, "lr": inner, "rr": outer},
                "FR": {"rf": inner, "lf": outer, "lr": outer, "rr": inner},
                "PR": {"rf": -pivot, "lf": pivot, "lr": pivot, "rr": -pivot},
            }
        )
        self.client.configure_drive(
            {
                "speed_mode": False,
                "straight_pwm": straight,
                "pivot_pwm": pivot,
                "curve_outer_pwm": outer,
                "curve_inner_pwm": inner,
                "profiles": profiles,
            },
        )
        self.stop()

    def apply(self, decision: PlannerDecision, observation: LineObservation) -> str:
        if (
            decision.intent is RouteIntent.STRAIGHT
            and decision.state is not RouteState.APPROACHING_RIGHT_CORNER
        ):
            drive_pwm = proportional_drive_pwm(observation, self.config)
            if drive_pwm is not None and self._direct_drive_supported:
                return self._apply_direct_drive(drive_pwm)

        action = action_for_decision(decision, observation, self.config)
        applied = self._apply_action(action)
        return f"P_FALLBACK:{applied}" if not self._direct_drive_supported else applied

    def _apply_direct_drive(self, drive_pwm: tuple[int, int]) -> str:
        drive_pwm = self._limit_pwm_step(drive_pwm)
        now = time.monotonic()
        if drive_pwm == self._last_action and now - self._last_sent_at < self.config.command_interval_seconds:
            return f"P(R={drive_pwm[0]},L={drive_pwm[1]})"
        try:
            self.client.send_drive_pwm(*drive_pwm)
        except RuntimeError as exc:
            if "HTTP Error 404" not in str(exc):
                raise
            # An older Pi robot-web service has no /api/drive endpoint. Keep
            # the vehicle controllable with the pre-P static profiles instead
            # of killing the vision loop.
            self._direct_drive_supported = False
            action = "F" if drive_pwm[0] == drive_pwm[1] else "FL" if drive_pwm[0] > drive_pwm[1] else "FR"
            return f"P_FALLBACK:{self._apply_action(action)}"
        self._last_action, self._last_sent_at = drive_pwm, now
        self._last_drive_pwm = drive_pwm
        return f"P(R={drive_pwm[0]},L={drive_pwm[1]})"

    def _limit_pwm_step(self, target: tuple[int, int]) -> tuple[int, int]:
        if self._last_drive_pwm is None:
            return target
        limit = self.config.pwm_max_step
        return tuple(
            max(previous - limit, min(previous + limit, requested))
            for previous, requested in zip(self._last_drive_pwm, target)
        )

    def _apply_action(self, action: str) -> str:
        now = time.monotonic()
        if action == self._last_action and now - self._last_sent_at < self.config.command_interval_seconds:
            return action
        self.client.send_action(action)
        self._last_action, self._last_sent_at = action, now
        return action

    def stop(self) -> None:
        try:
            self.client.stop()
        finally:
            self._last_action = "STOP"
            self._last_drive_pwm = None
            self._last_sent_at = time.monotonic()
