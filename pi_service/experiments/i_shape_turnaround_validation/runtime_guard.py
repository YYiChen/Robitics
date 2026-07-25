"""Safety checks shared by the isolated I-shape vehicle tests."""
from __future__ import annotations

from typing import Any


def require_no_competing_autonomous_route(status: dict[str, Any]) -> dict[str, Any]:
    """Reject a port-5000 service that owns an in-process route tracker.

    A disabled tracker does not write drive PWM, but its continuously running
    detector still consumes camera/CPU.  The I-shape validation is only
    repeatable when robot_web was started without --enable-autonomous-route.
    """
    robot = status.get("robot", {})
    autonomous = status.get("autonomous", {})
    if not robot.get("arduino_online"):
        raise RuntimeError("Arduino is not online; no motor command was sent")
    if autonomous.get("available"):
        enabled = autonomous.get("enabled")
        state = autonomous.get("state", "unknown")
        raise RuntimeError(
            "competing autonomous route service is present "
            f"(enabled={enabled}, state={state}); stop port-5000 and restart "
            "with ./start_robot.sh only (without --enable-autonomous-route)"
        )
    return status
