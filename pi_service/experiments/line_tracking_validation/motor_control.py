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
    straight_pwm: int = 105
    pivot_pwm: int = 165
    curve_inner_pwm: int = 60
    correction_deadband: float = 0.05
    command_interval_seconds: float = 0.18


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


class RobotWebMotorExecutor:
    """Configure and heartbeat the Pi ``/api/action`` control endpoint."""

    def __init__(self, config: MotorControlConfig) -> None:
        self.config = config
        self.client = RobotWebClient(RobotClientConfig(config.controller_url))
        self._last_action: str | None = None
        self._last_sent_at = 0.0

    def configure(self) -> None:
        """Require an online Arduino, then apply the requested PWM profiles."""
        status = self.client.require_arduino_online()
        robot = status.get("robot", {})
        current = robot.get("config", {})
        profiles = dict(current.get("profiles", {}))
        straight = self.config.straight_pwm
        inner = self.config.curve_inner_pwm
        pivot = self.config.pivot_pwm
        profiles.update(
            {
                "F": {"rf": straight, "lf": straight, "lr": straight, "rr": straight},
                "FL": {"rf": straight, "lf": inner, "lr": inner, "rr": straight},
                "FR": {"rf": inner, "lf": straight, "lr": straight, "rr": inner},
                "PR": {"rf": -pivot, "lf": pivot, "lr": pivot, "rr": -pivot},
            }
        )
        self.client.configure_drive(
            {
                "speed_mode": False,
                "straight_pwm": straight,
                "pivot_pwm": pivot,
                "curve_outer_pwm": straight,
                "curve_inner_pwm": inner,
                "profiles": profiles,
            },
        )
        self.stop()

    def apply(self, decision: PlannerDecision, observation: LineObservation) -> str:
        action = action_for_decision(decision, observation, self.config)
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
            self._last_sent_at = time.monotonic()
