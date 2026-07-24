"""HTTP motor executor for the existing Pi robot-web controller.

This adapter is deliberately separate from vision. It only becomes active when
the live monitor receives ``--enable-motors``.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import time
from typing import Protocol
from urllib.error import URLError
from urllib.request import Request, urlopen

from rectangle_route_planner import PlannerDecision, RouteIntent


class LineObservation(Protocol):
    offset: float | None


@dataclass(frozen=True)
class MotorControlConfig:
    controller_url: str
    straight_pwm: int = 120
    pivot_pwm: int = 150
    curve_inner_pwm: int = 80
    correction_deadband: float = 0.07
    command_interval_seconds: float = 0.18


def action_for_decision(
    decision: PlannerDecision,
    observation: LineObservation,
    config: MotorControlConfig,
) -> str:
    """Map safe route intent to the Pi controller's existing action names."""
    if decision.intent is RouteIntent.STOP or observation.offset is None:
        return "STOP"
    if decision.intent is RouteIntent.TURN_RIGHT:
        return "PR"
    if observation.offset < -config.correction_deadband:
        return "FL"
    if observation.offset > config.correction_deadband:
        return "FR"
    return "F"


class RobotWebMotorExecutor:
    """Configure and heartbeat the Pi ``/api/action`` control endpoint."""

    def __init__(self, config: MotorControlConfig) -> None:
        self.config = config
        self.base_url = config.controller_url.rstrip("/")
        self._last_action: str | None = None
        self._last_sent_at = 0.0

    def _request(self, path: str, payload: dict | None = None) -> dict:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(
            f"{self.base_url}{path}",
            data=data,
            method="GET" if payload is None else "POST",
            headers={"Content-Type": "application/json"} if payload is not None else {},
        )
        try:
            with urlopen(request, timeout=0.6) as response:
                decoded = json.loads(response.read().decode("utf-8"))
        except (URLError, OSError, ValueError) as exc:
            raise RuntimeError(f"robot controller request {path} failed: {exc}") from exc
        if not decoded.get("ok", True):
            raise RuntimeError(str(decoded.get("error", f"robot controller rejected {path}")))
        return decoded

    def configure(self) -> None:
        """Require an online Arduino, then apply the requested PWM profiles."""
        status = self._request("/api/status")
        robot = status.get("robot", {})
        if not robot.get("arduino_online"):
            raise RuntimeError("Arduino is not online; automatic driving was not armed")
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
        self._request(
            "/api/config",
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
        self._request("/api/action", {"action": action})
        self._last_action, self._last_sent_at = action, now
        return action

    def stop(self) -> None:
        try:
            self._request("/api/stop", {})
        finally:
            self._last_action = "STOP"
            self._last_sent_at = time.monotonic()
