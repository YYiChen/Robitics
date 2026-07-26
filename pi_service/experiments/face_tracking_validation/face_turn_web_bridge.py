"""Keep Pi face-pivot alive and stop it when the PC face result is centred.

The bridge is model-neutral.  It consumes the selected PC publisher's latest
JSON, so DeskMate YuNet/SFace or another isolated detector can be replaced
without duplicating Pi motor-control ownership.
"""
from __future__ import annotations

import argparse
from collections.abc import Callable
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any, TextIO
from urllib.request import Request, urlopen
from uuid import uuid4

DEFAULT_FACE_DEADBAND_NORMALIZED = .30
FACE_COMMAND_ACTIONS = {
    "HEARTBEAT": "face_turn_heartbeat",
    "STOP": "face_turn_stop",
}


class FaceStopArmer:
    """Ignore an initially centred face until the turn has moved away from it."""

    def __init__(self) -> None:
        self.armed = False

    def reset(self) -> None:
        self.armed = False

    def should_stop(self, centred: bool) -> bool:
        if not centred:
            self.armed = True
            return False
        return self.armed


def decode_json_object(body: bytes) -> dict:
    """Decode direct JSON objects and the Pi's legacy double-encoded response."""

    value = json.loads(body.decode("utf-8"))
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict):
        raise ValueError("expected a JSON object")
    return value


def fetch_json(url: str, timeout: float = .4) -> dict:
    with urlopen(url, timeout=timeout) as response:
        return decode_json_object(response.read())


def robotics_route_status(payload: dict) -> dict:
    """Extract route state from the versioned robotics status envelope."""

    status = payload.get("status")
    if not payload.get("ok") or not isinstance(status, dict):
        raise ValueError("invalid robotics-v1 status envelope")
    route = status.get("route")
    if not isinstance(route, dict):
        raise ValueError("robotics-v1 status has no route object")
    return route


def robotics_action_payload(command: str, request_id: str) -> dict:
    """Map bridge decisions onto the versioned state-machine action contract."""

    normalized = str(command).strip().upper()
    try:
        action = FACE_COMMAND_ACTIONS[normalized]
    except KeyError as exc:
        raise ValueError("bridge command must be HEARTBEAT or STOP") from exc
    return {"request_id": request_id, "action": action}


def post_command(pi_url: str, command: str, timeout: float = .4) -> dict:
    normalized = str(command).strip().upper()
    request_id = f"face-bridge-{normalized.lower()}-{uuid4().hex}"
    body = json.dumps(
        robotics_action_payload(normalized, request_id),
        separators=(",", ":"),
    ).encode("utf-8")
    request = Request(
        f"{pi_url.rstrip('/')}/api/robotics/v1/actions",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=timeout) as response:
        return decode_json_object(response.read())


def payload_age_ms(payload: dict) -> float | None:
    try:
        captured = datetime.fromisoformat(
            str(payload.get("time")).replace("Z", "+00:00")
        )
        return (datetime.now(timezone.utc) - captured).total_seconds() * 1000
    except (TypeError, ValueError):
        return None


def is_fresh_payload(payload: dict, *, max_age_ms: int) -> bool:
    age_ms = payload_age_ms(payload)
    return age_ms is not None and 0 <= age_ms <= max_age_ms


def is_fresh_and_centred(face: dict, *, minimum_score: float, deadband_normalized: float,
                          max_age_ms: int) -> bool:
    if not face.get("detected") or float(face.get("score", 0.0)) < minimum_score:
        return False
    offset = face.get("offset_x_normalized")
    if offset is None or abs(float(offset)) > deadband_normalized:
        return False
    return is_fresh_payload(face, max_age_ms=max_age_ms)


class FaceTurnBridge:
    """Translate one face observation source into Pi robotics-v1 lease actions."""

    def __init__(
        self,
        *,
        face_provider: Callable[[], dict],
        pi_url: str,
        heartbeat_seconds: float = .18,
        minimum_score: float = .5,
        deadband_normalized: float = DEFAULT_FACE_DEADBAND_NORMALIZED,
        max_age_ms: int = 450,
        status_provider: Callable[[], dict] | None = None,
        command_poster: Callable[[str, str], dict] | None = None,
    ) -> None:
        self.face_provider = face_provider
        self.pi_url = pi_url.rstrip("/")
        self.heartbeat_seconds = heartbeat_seconds
        self.minimum_score = minimum_score
        self.deadband_normalized = deadband_normalized
        self.max_age_ms = max_age_ms
        self.status_provider = status_provider or (
            lambda: fetch_json(f"{self.pi_url}/api/robotics/v1/status")
        )
        self.command_poster = command_poster or (
            lambda pi_url, command: post_command(pi_url, command)
        )
        self.face_stop_armer = FaceStopArmer()
        self.was_face_turn_active = False

    def step(self) -> dict[str, Any]:
        record: dict[str, Any] = {
            "time": datetime.now(timezone.utc).isoformat()
        }
        status = self.status_provider()
        route = robotics_route_status(status)
        face_turn_active = route.get("motion_phase") == "FACE_CENTER_TURN"
        if not face_turn_active:
            self.face_stop_armer.reset()
            self.was_face_turn_active = False
            record.update(action="idle", reason=route.get("motion_phase"))
            return record

        if not self.was_face_turn_active:
            self.face_stop_armer.reset()
            self.was_face_turn_active = True
        face = self.face_provider()
        if face.get("error"):
            raise RuntimeError(f"face_publisher_error:{face['error']}")
        if not is_fresh_payload(face, max_age_ms=self.max_age_ms):
            raise RuntimeError("face_publisher_stale")
        centred = is_fresh_and_centred(
            face,
            minimum_score=self.minimum_score,
            deadband_normalized=self.deadband_normalized,
            max_age_ms=self.max_age_ms,
        )
        if self.face_stop_armer.should_stop(centred):
            reply = self.command_poster(self.pi_url, "STOP")
            action = "stop_centered"
        else:
            reply = self.command_poster(self.pi_url, "HEARTBEAT")
            action = "heartbeat"
        record.update(
            action=action,
            face=face,
            face_centred=centred,
            face_stop_armed=self.face_stop_armer.armed,
            reply=reply,
        )
        return record

    def run_forever(
        self,
        log: TextIO,
        *,
        record_callback: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        while True:
            try:
                record = self.step()
            except Exception as exc:
                # Deliberately do not send HEARTBEAT on any camera/network
                # fault: the Pi-side dead-man timer performs STOP.
                record = {
                    "time": datetime.now(timezone.utc).isoformat(),
                    "action": "no_heartbeat",
                    "error": str(exc),
                }
            if record_callback is not None:
                record_callback(dict(record))
            log.write(json.dumps(record, ensure_ascii=False) + "\n")
            log.flush()
            time.sleep(self.heartbeat_seconds)


def main() -> None:
    parser = argparse.ArgumentParser(description="PC bridge for webpage J/L face pivot")
    parser.add_argument("--face-url", default="http://127.0.0.1:5059/api/face/latest")
    parser.add_argument("--pi-url", default="http://100.80.46.54:5000")
    parser.add_argument("--heartbeat-seconds", type=float, default=.18)
    parser.add_argument("--minimum-score", type=float, default=.5)
    # Stop once the face centre is within ±30% of the half-frame width.
    # This is about ±192 px for the current 1280 px DroidCam stream and covers
    # the observed near-centre position before the next pulse overshoots it.
    parser.add_argument("--deadband-normalized", type=float, default=DEFAULT_FACE_DEADBAND_NORMALIZED)
    parser.add_argument("--max-age-ms", type=int, default=450)
    args = parser.parse_args()
    log_dir = Path(__file__).with_name("runtime_logs"); log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"face_turn_web_bridge_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.jsonl"
    print(f"Web J/L bridge running: face={args.face_url}, Pi={args.pi_url}")
    print(
        "It never starts a turn. It uses /api/robotics/v1/actions only to "
        "heartbeat an active Pi face turn and stop it once centred."
    )
    bridge = FaceTurnBridge(
        face_provider=lambda: fetch_json(args.face_url),
        pi_url=args.pi_url,
        heartbeat_seconds=args.heartbeat_seconds,
        minimum_score=args.minimum_score,
        deadband_normalized=args.deadband_normalized,
        max_age_ms=args.max_age_ms,
    )
    try:
        with log_path.open("a", encoding="utf-8") as log:
            bridge.run_forever(log)
    except KeyboardInterrupt:
        print("Bridge stopped; Pi dead-man timer will stop any active pivot.")


if __name__ == "__main__":
    main()
