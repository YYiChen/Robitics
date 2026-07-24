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

    def stop(self) -> None:
        self._request("/api/stop", {})

    def configure_drive(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Persist a validated drive profile through robot-web's API."""
        return dict(self._request("/api/config", payload)["config"])
