"""Keep Pi face-pivot alive and stop it when the PC face result is centred.

The bridge is model-neutral.  It consumes the selected PC publisher's latest
JSON, so DeskMate YuNet/SFace or another isolated detector can be replaced
without duplicating Pi motor-control ownership.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen
from uuid import uuid4

DEFAULT_FACE_DEADBAND_NORMALIZED = .20
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


def is_fresh_and_centred(face: dict, *, minimum_score: float, deadband_normalized: float,
                          max_age_ms: int) -> bool:
    if not face.get("detected") or float(face.get("score", 0.0)) < minimum_score:
        return False
    offset = face.get("offset_x_normalized")
    if offset is None or abs(float(offset)) > deadband_normalized:
        return False
    try:
        captured = datetime.fromisoformat(str(face.get("time")).replace("Z", "+00:00"))
        age_ms = (datetime.now(timezone.utc) - captured).total_seconds() * 1000
    except (TypeError, ValueError):
        return False
    return 0 <= age_ms <= max_age_ms


def main() -> None:
    parser = argparse.ArgumentParser(description="PC bridge for webpage J/L face pivot")
    parser.add_argument("--face-url", default="http://127.0.0.1:5059/api/face/latest")
    parser.add_argument("--pi-url", default="http://100.80.46.54:5000")
    parser.add_argument("--heartbeat-seconds", type=float, default=.18)
    parser.add_argument("--minimum-score", type=float, default=.5)
    # Stop once the face centre is within ±20% of the half-frame width.
    # This is about ±128 px for the current 1280 px DroidCam stream and covers
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
    face_stop_armer = FaceStopArmer()
    was_face_turn_active = False
    try:
        with log_path.open("a", encoding="utf-8") as log:
            while True:
                record = {"time": datetime.now(timezone.utc).isoformat()}
                try:
                    status = fetch_json(f"{args.pi_url.rstrip('/')}/api/status")
                    autonomous = status.get("autonomous", {})
                    face_turn_active = autonomous.get("motion_phase") == "FACE_CENTER_TURN"
                    if not face_turn_active:
                        face_stop_armer.reset()
                        was_face_turn_active = False
                        record.update(action="idle", reason=autonomous.get("motion_phase"))
                    else:
                        if not was_face_turn_active:
                            face_stop_armer.reset()
                            was_face_turn_active = True
                        face = fetch_json(args.face_url)
                        centred = is_fresh_and_centred(
                            face,
                            minimum_score=args.minimum_score,
                            deadband_normalized=args.deadband_normalized,
                            max_age_ms=args.max_age_ms,
                        )
                        if face_stop_armer.should_stop(centred):
                            reply, action = post_command(args.pi_url, "STOP"), "stop_centered"
                        else:
                            reply, action = post_command(args.pi_url, "HEARTBEAT"), "heartbeat"
                        record.update(action=action, face=face, face_centred=centred, face_stop_armed=face_stop_armer.armed, reply=reply)
                except Exception as exc:
                    # Deliberately do not send HEARTBEAT on any local/network
                    # fault: the Pi-side 0.6 s dead-man timer performs STOP.
                    record.update(action="no_heartbeat", error=str(exc))
                log.write(json.dumps(record, ensure_ascii=False) + "\n"); log.flush()
                time.sleep(args.heartbeat_seconds)
    except KeyboardInterrupt:
        print("Bridge stopped; Pi dead-man timer will stop any active pivot.")


if __name__ == "__main__":
    main()
