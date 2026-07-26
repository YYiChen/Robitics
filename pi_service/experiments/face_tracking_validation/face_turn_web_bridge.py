"""Lightweight PC bridge: keep Pi face-pivot alive and stop it when centred.

MediaPipe remains exclusively inside ``face_position_server.py``.  This process
only consumes its latest JSON, so the UI can trigger J/L on the Pi while the
expensive model keeps running independently.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen


def fetch_json(url: str, timeout: float = .4) -> dict:
    with urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def post_command(pi_url: str, command: str, timeout: float = .4) -> dict:
    body = json.dumps({"command": command}).encode("utf-8")
    request = Request(f"{pi_url.rstrip('/')}/api/autonomous/face-turn", data=body,
                      headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


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
    # ±8% of half-frame width: about ±26 px for the 640 px Pi stream.
    parser.add_argument("--deadband-normalized", type=float, default=.08)
    parser.add_argument("--max-age-ms", type=int, default=450)
    args = parser.parse_args()
    log_dir = Path(__file__).with_name("runtime_logs"); log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"face_turn_web_bridge_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.jsonl"
    print(f"Web J/L bridge running: face={args.face_url}, Pi={args.pi_url}")
    print("It never starts a turn. It only heartbeats an active Pi face turn and stops it once centred.")
    try:
        with log_path.open("a", encoding="utf-8") as log:
            while True:
                record = {"time": datetime.now(timezone.utc).isoformat()}
                try:
                    status = fetch_json(f"{args.pi_url.rstrip('/')}/api/status")
                    autonomous = status.get("autonomous", {})
                    if autonomous.get("motion_phase") != "FACE_CENTER_TURN":
                        record.update(action="idle", reason=autonomous.get("motion_phase"))
                    else:
                        face = fetch_json(args.face_url)
                        if is_fresh_and_centred(face, minimum_score=args.minimum_score,
                                                deadband_normalized=args.deadband_normalized,
                                                max_age_ms=args.max_age_ms):
                            reply, action = post_command(args.pi_url, "STOP"), "stop_centered"
                        else:
                            reply, action = post_command(args.pi_url, "HEARTBEAT"), "heartbeat"
                        record.update(action=action, face=face, reply=reply)
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
