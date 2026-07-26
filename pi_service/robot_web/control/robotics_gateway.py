"""Versioned semantic facade for an external robotics state machine."""

from __future__ import annotations

from collections import OrderedDict
from datetime import datetime, timezone
import json
import re
import threading
from typing import Any, Mapping


ROBOTICS_API_VERSION = "1.0"
MAX_REMEMBERED_REQUESTS = 128
REQUEST_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,159}$")

LINE_FOLLOW_TUNING_KEYS = (
    "process_fps",
    "straight_pwm",
    "correction_deadband",
    "correction_gain",
    "minimum_correction_pwm",
    "maximum_correction_pwm",
    "line_lost_confirm_frames",
    "brake_hold_seconds",
)
VISUAL_TURN_TUNING_KEYS = (
    "face_turn_pwm",
    "face_turn_pulse_seconds",
    "face_turn_cooldown_seconds",
    "face_turn_max_seconds",
    "face_turn_line_center_deadband_normalized",
    "face_turn_line_center_confirm_frames",
)
CARD_DEFAULTS = {
    "feed_pwm": -150,
    "feed_duration_ms": 1500,
    "deal_pwm": 150,
    "deal_duration_ms": 400,
}


class RoboticsGateway:
    """Map stable semantic requests onto the existing single-owner services."""

    ACTIONS = (
        "follow_line_to_end",
        "face_turn_start",
        "face_turn_heartbeat",
        "face_turn_stop",
        "line_recenter_start",
        "line_recenter_stop",
        "preset_turn",
        "dispense_one",
        "stop",
    )

    def __init__(self, controller: Any, route_tracker: Any) -> None:
        self.controller = controller
        self.route_tracker = route_tracker
        self._lock = threading.RLock()
        self._remembered: OrderedDict[str, tuple[str, dict[str, Any]]] = OrderedDict()
        self._active_route_request_id: str | None = None
        self._active_dispense_request_id: str | None = None

    def capabilities(self) -> dict[str, Any]:
        route_available = self.route_tracker is not None and all(
            hasattr(self.route_tracker, name)
            for name in (
                "request_follow_to_end",
                "request_face_center_turn",
                "request_line_center_turn",
                "request_roundtrip_stop",
                "set_drive_enabled",
                "status_dict",
                "update_tuning",
            )
        )
        controller_available = self.controller is not None and all(
            hasattr(self.controller, name)
            for name in (
                "deal_from_key_request",
                "deal_request_status",
                "status",
                "stop_now",
            )
        )
        return {
            "api_version": ROBOTICS_API_VERSION,
            "available": route_available and controller_available,
            "route_available": route_available,
            "controller_available": controller_available,
            "actions": [
                action
                for action in self.ACTIONS
                if action != "preset_turn"
                or (
                    self.route_tracker is not None
                    and hasattr(self.route_tracker, "request_manual_turn")
                )
            ],
            "motor_gate": "explicit_boolean",
            "request_idempotency": True,
            "request_terminal_status": True,
            "request_status_values": [
                "running",
                "succeeded",
                "failed",
                "cancelled",
            ],
            "dispense_completion_evidence": "arduino_command_ack_only",
            "physical_card_exit_verified": False,
        }

    def status(self) -> dict[str, Any]:
        route = (
            self.route_tracker.status_dict()
            if self.route_tracker is not None and hasattr(self.route_tracker, "status_dict")
            else {"available": False, "enabled": False, "state": "disabled"}
        )
        robot = (
            self.controller.status()
            if self.controller is not None and hasattr(self.controller, "status")
            else {"serial": False, "arduino_online": False}
        )
        with self._lock:
            self._refresh_active_route_request(route)
            self._refresh_active_dispense_request()
        return {
            "api_version": ROBOTICS_API_VERSION,
            "available": self.capabilities()["available"],
            "gate_enabled": bool(route.get("enabled", False)),
            "route": route,
            "robot": robot,
        }

    def set_gate(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        self._require_available()
        self._reject_unknown(payload, {"enabled"}, "gate")
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        route = self.route_tracker.set_drive_enabled(enabled)
        return {
            "api_version": ROBOTICS_API_VERSION,
            "enabled": bool(route.get("enabled", enabled)),
            "route": route,
        }

    def config(self) -> dict[str, Any]:
        self._require_available()
        tuning = self.route_tracker.status_dict().get("tuning", {})
        return {
            "api_version": ROBOTICS_API_VERSION,
            "line_follow": {
                key: tuning[key] for key in LINE_FOLLOW_TUNING_KEYS if key in tuning
            },
            "visual_turn": {
                key: tuning[key] for key in VISUAL_TURN_TUNING_KEYS if key in tuning
            },
            "dispense_defaults": dict(CARD_DEFAULTS),
        }

    def update_config(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        self._require_available()
        self._reject_unknown(payload, {"line_follow", "visual_turn"}, "config")
        flattened: dict[str, Any] = {}
        for section, allowed in (
            ("line_follow", set(LINE_FOLLOW_TUNING_KEYS)),
            ("visual_turn", set(VISUAL_TURN_TUNING_KEYS)),
        ):
            values = payload.get(section, {})
            if not isinstance(values, Mapping):
                raise ValueError(f"{section} must be an object")
            self._reject_unknown(values, allowed, section)
            flattened.update(values)
        if not flattened:
            raise ValueError("config update must contain at least one value")
        self.route_tracker.update_tuning(flattened)
        return self.config()

    def execute(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        self._require_available()
        allowed = {
            "request_id",
            "action",
            "direction",
            "degrees",
            "feed_pwm",
            "feed_duration_ms",
            "deal_pwm",
            "deal_duration_ms",
        }
        self._reject_unknown(payload, allowed, "action")
        request_id = self._normalize_request_id(payload.get("request_id"))
        action = str(payload.get("action", "")).strip().lower()
        if action not in self.ACTIONS:
            raise ValueError(f"unsupported action: {action}")
        fingerprint = json.dumps(dict(payload), sort_keys=True, separators=(",", ":"))

        with self._lock:
            remembered = self._remembered.get(request_id)
            if remembered is not None:
                old_fingerprint, response = remembered
                if old_fingerprint != fingerprint:
                    raise ValueError("request_id was already used with a different payload")
                if action == "dispense_one":
                    response = self._refresh_request(
                        request_id, old_fingerprint, response
                    )
                return dict(response)

            response = self._dispatch(action, request_id, payload)
            self._settle_superseded_request(action, request_id)
            self._remember(request_id, fingerprint, response)
            if response["request_status"] == "running":
                if action == "dispense_one":
                    self._active_dispense_request_id = request_id
                elif action in {
                    "follow_line_to_end",
                    "face_turn_start",
                    "line_recenter_start",
                    "preset_turn",
                }:
                    self._active_route_request_id = request_id
            return response

    def request_result(self, request_id: object) -> dict[str, Any] | None:
        """Resolve a timed-out call without reissuing its physical action."""

        normalized = self._normalize_request_id(request_id)
        with self._lock:
            remembered = self._remembered.get(normalized)
            if remembered is None:
                return None
            fingerprint, response = remembered
            response = self._refresh_request(normalized, fingerprint, response)
            return dict(response)

    def _dispatch(
        self, action: str, request_id: str, payload: Mapping[str, Any]
    ) -> dict[str, Any]:
        direction = str(payload.get("direction", "")).strip().upper()
        if action in {"face_turn_start", "line_recenter_start", "preset_turn"}:
            if direction not in {"LEFT", "RIGHT"}:
                raise ValueError(f"{action} requires direction LEFT or RIGHT")
        elif direction:
            raise ValueError(f"{action} does not accept direction")
        degrees = payload.get("degrees")
        if action == "preset_turn":
            if isinstance(degrees, bool) or not isinstance(degrees, int):
                raise ValueError("preset_turn requires degrees 90 or 180") from None
            if degrees not in {90, 180}:
                raise ValueError("preset_turn requires degrees 90 or 180")
        elif degrees is not None:
            raise ValueError(f"{action} does not accept degrees")

        detail: Any
        if action == "follow_line_to_end":
            detail = self.route_tracker.request_follow_to_end()
        elif action == "face_turn_start":
            detail = self.route_tracker.request_face_center_turn(f"START_{direction}")
        elif action == "face_turn_heartbeat":
            detail = self.route_tracker.request_face_center_turn("HEARTBEAT")
        elif action == "face_turn_stop":
            detail = self.route_tracker.request_face_center_turn("STOP")
        elif action == "line_recenter_start":
            detail = self.route_tracker.request_line_center_turn(f"START_{direction}")
        elif action == "line_recenter_stop":
            detail = self.route_tracker.request_line_center_turn("STOP")
        elif action == "preset_turn":
            if not hasattr(self.route_tracker, "request_manual_turn"):
                raise RuntimeError("preset turn is not available for this route")
            detail = self.route_tracker.request_manual_turn(
                f"{direction}_{degrees}"
            )
        elif action == "dispense_one":
            request = {"token": request_id}
            for key, default in CARD_DEFAULTS.items():
                request[key] = default
            detail = self.controller.deal_from_key_request(request)
        else:
            detail = self.route_tracker.request_roundtrip_stop()
            self.controller.stop_now()

        return self._action_response(action, request_id, detail)

    def _action_response(
        self,
        action: str,
        request_id: str,
        detail: Any,
        *,
        accepted_at: object = None,
    ) -> dict[str, Any]:
        now = datetime.now(timezone.utc).isoformat()
        response = {
            "api_version": ROBOTICS_API_VERSION,
            "request_id": request_id,
            "action": action,
            "accepted": True,
            "accepted_at": str(accepted_at or now),
            "updated_at": now,
            "state": (
                detail.get("state", "accepted")
                if isinstance(detail, Mapping)
                else "accepted"
            ),
            "detail": detail,
        }
        request_status = self._classify_request_status(action, detail)
        response["request_status"] = request_status
        response["terminal"] = request_status in {
            "succeeded",
            "failed",
            "cancelled",
        }
        if response["terminal"]:
            response["completed_at"] = now
        if action == "dispense_one":
            response.update(
                {
                    "completion_evidence": "arduino_command_ack_only",
                    "physical_card_exit_verified": False,
                }
            )
        return response

    def _refresh_request(
        self,
        request_id: str,
        fingerprint: str,
        response: dict[str, Any],
    ) -> dict[str, Any]:
        if response.get("terminal") is True:
            return response
        action = str(response.get("action", ""))
        detail: Any = None
        if action == "dispense_one":
            detail = self.controller.deal_request_status(request_id)
        elif request_id == self._active_route_request_id:
            detail = self.route_tracker.status_dict()
        if detail is None:
            return response
        refreshed = self._action_response(
            action,
            request_id,
            detail,
            accepted_at=response.get("accepted_at"),
        )
        self._remember(request_id, fingerprint, refreshed)
        if refreshed["terminal"]:
            if request_id == self._active_route_request_id:
                self._active_route_request_id = None
            if request_id == self._active_dispense_request_id:
                self._active_dispense_request_id = None
        return refreshed

    def _refresh_active_route_request(self, route: Mapping[str, Any]) -> None:
        request_id = self._active_route_request_id
        if request_id is None:
            return
        remembered = self._remembered.get(request_id)
        if remembered is None:
            self._active_route_request_id = None
            return
        fingerprint, response = remembered
        refreshed = self._action_response(
            str(response["action"]),
            request_id,
            route,
            accepted_at=response.get("accepted_at"),
        )
        self._remember(request_id, fingerprint, refreshed)
        if refreshed["terminal"]:
            self._active_route_request_id = None

    def _refresh_active_dispense_request(self) -> None:
        request_id = self._active_dispense_request_id
        if request_id is None:
            return
        remembered = self._remembered.get(request_id)
        if remembered is None:
            self._active_dispense_request_id = None
            return
        fingerprint, response = remembered
        self._refresh_request(request_id, fingerprint, response)

    def _settle_superseded_request(self, action: str, request_id: str) -> None:
        previous = self._active_route_request_id
        if previous is not None and action != "face_turn_heartbeat":
            previous_action = str(self._remembered[previous][1].get("action", ""))
            succeeded = (
                action == "face_turn_stop" and previous_action == "face_turn_start"
            ) or (
                action == "line_recenter_stop"
                and previous_action == "line_recenter_start"
            )
            self._finish_remembered(
                previous,
                "succeeded" if succeeded else "cancelled",
                completed_by_request_id=request_id,
            )
            self._active_route_request_id = None
        if action == "dispense_one":
            previous_dispense = self._active_dispense_request_id
            if previous_dispense is not None:
                self._finish_remembered(
                    previous_dispense,
                    "cancelled",
                    completed_by_request_id=request_id,
                )
                self._active_dispense_request_id = None

    def _finish_remembered(
        self,
        request_id: str,
        request_status: str,
        *,
        completed_by_request_id: str,
    ) -> None:
        remembered = self._remembered.get(request_id)
        if remembered is None:
            return
        fingerprint, response = remembered
        now = datetime.now(timezone.utc).isoformat()
        updated = {
            **response,
            "updated_at": now,
            "request_status": request_status,
            "terminal": True,
            "completed_at": now,
            "completed_by_request_id": completed_by_request_id,
        }
        self._remember(request_id, fingerprint, updated)

    @staticmethod
    def _classify_request_status(action: str, detail: Any) -> str:
        state = (
            str(detail.get("state", "")).strip().upper()
            if isinstance(detail, Mapping)
            else ""
        )
        if any(marker in state for marker in ("ERROR", "FAULT", "LOST", "TIMEOUT")):
            return "failed"
        if action == "dispense_one":
            if state in {"ERROR", "PARTIAL", "FAILED"}:
                return "failed"
            if state == "COMPLETED" or (
                state == "RUNNING"
                and isinstance(detail, Mapping)
                and "feed" in detail
                and "deal" in detail
            ):
                return "succeeded"
            return "running"
        if action in {
            "face_turn_heartbeat",
            "face_turn_stop",
            "line_recenter_stop",
            "stop",
        }:
            return "succeeded"
        success_states = {
            "follow_line_to_end": {"END_REACHED"},
            "face_turn_start": {"FACE_CENTERED_STOP"},
            "line_recenter_start": {"LINE_TURN_CENTERED", "LINE_TURN_STOPPED"},
            "preset_turn": {
                "MANUAL_RED_ALIGNED",
                "MANUAL_COMPLETE",
                "TURN_COMPLETE",
            },
        }
        if state in success_states.get(action, set()):
            return "succeeded"
        return "running"

    def _remember(
        self, request_id: str, fingerprint: str, response: dict[str, Any]
    ) -> None:
        self._remembered[request_id] = (fingerprint, dict(response))
        self._remembered.move_to_end(request_id)
        while len(self._remembered) > MAX_REMEMBERED_REQUESTS:
            self._remembered.popitem(last=False)

    def _require_available(self) -> None:
        if not self.capabilities()["available"]:
            raise RuntimeError("end-line robotics route is not available")

    @staticmethod
    def _normalize_request_id(value: object) -> str:
        normalized = str(value or "").strip()
        if not REQUEST_ID_PATTERN.fullmatch(normalized):
            raise ValueError(
                "request_id must be 1 to 160 URL-safe characters "
                "(letters, digits, dot, underscore, colon or hyphen)"
            )
        return normalized

    @staticmethod
    def _reject_unknown(
        payload: Mapping[str, Any], allowed: set[str], label: str
    ) -> None:
        if not isinstance(payload, Mapping):
            raise ValueError(f"{label} payload must be an object")
        unknown = set(payload) - allowed
        if unknown:
            raise ValueError(f"unknown {label} fields: {sorted(unknown)}")


__all__ = [
    "CARD_DEFAULTS",
    "LINE_FOLLOW_TUNING_KEYS",
    "REQUEST_ID_PATTERN",
    "ROBOTICS_API_VERSION",
    "RoboticsGateway",
    "VISUAL_TURN_TUNING_KEYS",
]
