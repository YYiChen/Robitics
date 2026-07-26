"""Shared client for the Pi robot-web motor service.

``robot_web`` is the only process allowed to own the Arduino serial port.
Other Pi-service programs import this module and call its HTTP API instead.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any
from urllib.error import URLError
from urllib.request import Request, urlopen


ACTIONS = frozenset({"STOP", "F", "SF", "B", "PL", "PR", "SPL", "SPR", "FL", "FR", "BL", "BR"})


@dataclass(frozen=True)
class RobotClientConfig:
    base_url: str = "http://127.0.0.1:5000"
    timeout_seconds: float = 0.6


class RobotWebClient:
    """Safe common API for programs that need to command the robot."""

    def __init__(self, config: RobotClientConfig = RobotClientConfig()) -> None:
        self.config = config
        self.base_url = config.base_url.rstrip("/")

    def _request(self, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(
            f"{self.base_url}{path}",
            data=data,
            method="GET" if payload is None else "POST",
            headers={"Content-Type": "application/json"} if payload is not None else {},
        )
        try:
            with urlopen(request, timeout=self.config.timeout_seconds) as response:
                decoded = json.loads(response.read().decode("utf-8"))
        except (URLError, OSError, ValueError) as exc:
            raise RuntimeError(f"robot controller request {path} failed: {exc}") from exc
        if not decoded.get("ok", True):
            raise RuntimeError(str(decoded.get("error", f"robot controller rejected {path}")))
        return decoded

    def status(self) -> dict[str, Any]:
        return self._request("/api/status")

    def require_arduino_online(self) -> dict[str, Any]:
        status = self.status()
        if not status.get("robot", {}).get("arduino_online"):
            raise RuntimeError("Arduino is not online; motor control was not armed")
        return status

    def send_action(self, action: str) -> str:
        action = action.upper()
        if action not in ACTIONS:
            raise ValueError(f"unsupported robot action: {action}")
        return str(self._request("/api/action", {"action": action})["action"])

    def send_drive_pwm(self, right_pwm: int, left_pwm: int) -> tuple[int, int]:
        """Send one bounded, non-persistent M1/M2 differential command."""
        if not -255 <= right_pwm <= 255 or not -255 <= left_pwm <= 255:
            raise ValueError("drive PWM must be in [-255, 255]")
        response = self._request(
            "/api/drive",
            {"right_pwm": right_pwm, "left_pwm": left_pwm},
        )
        return int(response["right_pwm"]), int(response["left_pwm"])

    def stop(self) -> None:
        self._request("/api/stop", {})

    def start_face_turn(self, direction: str) -> dict[str, Any]:
        direction = direction.upper()
        if direction not in {"LEFT", "RIGHT"}:
            raise ValueError("face turn direction must be LEFT or RIGHT")
        response = self._request(
            "/api/autonomous/face-turn",
            {"action": "START", "direction": direction},
        )
        return dict(response["autonomous"])

    def send_face_observation(
        self,
        *,
        found: bool,
        frame_width: int,
        center_x: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "action": "OBSERVE",
            "found": bool(found),
            "frame_width": int(frame_width),
        }
        if found:
            if center_x is None:
                raise ValueError("center_x is required when a face is found")
            payload["center_x"] = float(center_x)
        response = self._request("/api/autonomous/face-turn", payload)
        return dict(response["autonomous"])

    def cancel_face_turn(self) -> dict[str, Any]:
        response = self._request("/api/autonomous/face-turn", {"action": "CANCEL"})
        return dict(response["autonomous"])

    def configure_drive(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Persist a validated drive profile through robot-web's API."""
        return dict(self._request("/api/config", payload)["config"])
