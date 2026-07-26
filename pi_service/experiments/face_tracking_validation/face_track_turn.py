"""PC side: read Pi MJPEG → MediaPipe face detect → send LEFT_90/RIGHT_90 to Pi.

Single OpenCV window.  No web browser.  No HTTP server on PC.
All motor commands are logged to both the console and a JSONL file.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import urllib.request
import urllib.error

import cv2
import numpy as np

from face_detector import FaceDetector

HERE = Path(__file__).resolve().parent
LOG_DIR = HERE / "runtime_logs"

# ── Pi API helpers ──────────────────────────────────────────────────────────

class PiMotorClient:
    """Minimal HTTP client for the Pi end_line_turn_adaptor API."""

    def __init__(self, pi_url: str, log_path: Path) -> None:
        self.pi_url = pi_url.rstrip("/")
        self._log_path = log_path

    def _log(self, kind: str, extra: dict | None = None) -> None:
        record = {"time_utc": datetime.now(timezone.utc).isoformat(), "kind": kind}
        if extra:
            record.update(extra)
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._log_path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _get(self, path: str, timeout: float = 3.0) -> dict | None:
        try:
            req = urllib.request.Request(f"{self.pi_url}{path}")
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read())
        except urllib.error.URLError as exc:
            self._log("pi_get_error", {"path": path, "error": str(exc)})
            return None
        except Exception as exc:
            self._log("pi_get_exception", {"path": path, "error": str(exc)})
            return None

    def _post(self, path: str, body: dict | None = None, timeout: float = 3.0) -> dict | None:
        data = json.dumps(body).encode("utf-8") if body else None
        headers = {"Content-Type": "application/json"} if body else {}
        try:
            req = urllib.request.Request(f"{self.pi_url}{path}", data=data, headers=headers, method="POST")
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return json.loads(resp.read())
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            self._log("pi_post_http_error", {"path": path, "status": exc.code, "detail": detail})
            return None
        except urllib.error.URLError as exc:
            self._log("pi_post_url_error", {"path": path, "error": str(exc)})
            return None
        except Exception as exc:
            self._log("pi_post_exception", {"path": path, "error": str(exc)})
            return None

    def status(self) -> dict | None:
        return self._get("/api/status", timeout=3.0)

    def toggle_m(self) -> bool:
        """Arm the motor gate.  Returns True if M is now enabled."""
        # check current state first
        s = self.status()
        if s and s.get("autonomous", {}).get("enabled"):
            self._log("pi_m_gate", {"action": "already_enabled"})
            return True

        result = self._post("/api/autonomous/toggle")
        if result and result.get("autonomous", {}).get("enabled"):
            self._log("pi_m_gate", {"action": "toggled_on"})
            return True
        self._log("pi_m_gate", {"action": "toggle_failed", "response": result})
        return False

    def send_turn(self, direction: str) -> bool:
        """Send LEFT_90 or RIGHT_90.  Returns True if Pi accepted it."""
        start_ns = time.monotonic_ns()
        result = self._post("/api/autonomous/manual-turn", {"command": direction})
        elapsed_ms = round((time.monotonic_ns() - start_ns) / 1_000_000, 1)
        accepted = result is not None and result.get("ok") is True
        self._log("pi_turn_command", {
            "direction": direction, "accepted": accepted,
            "rtt_ms": elapsed_ms, "response": result,
        })
        return accepted


# ── Main loop ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="PC face-offset → Pi in-place pivot")
    parser.add_argument("--pi-source", default="http://10.157.23.223:4747/video")
    parser.add_argument("--pi-url", default="http://100.80.46.54:5000")
    parser.add_argument("--deadband-px", type=int, default=60)
    parser.add_argument("--cooldown-seconds", type=float, default=0.8)
    parser.add_argument("--no-motor", action="store_true")
    parser.add_argument("--confidence", type=float, default=0.5)
    args = parser.parse_args()

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = LOG_DIR / f"face_track_turn_{stamp}.jsonl"
    pi = PiMotorClient(args.pi_url, log_path) if not args.no_motor else None

    print("=" * 60)
    print("PC Face → Pi Turn")
    print(f"  Pi stream:   {args.pi_source}")
    if pi:
        print(f"  Pi API:      {args.pi_url}")
        print(f"  Deadband:    {args.deadband_px} px")
        print(f"  Cooldown:    {args.cooldown_seconds} s")
    else:
        print("  Motor:       DISABLED (--no-motor)")
    print(f"  Log:         {log_path}")
    print("=" * 60)

    # Check Pi connectivity
    if pi:
        status = pi.status()
        if status is None:
            print("\n*** WARNING: Pi not reachable at", args.pi_url)
            print("*** Motor commands will fail until Pi comes online.\n")
        else:
            a = status.get("autonomous", {})
            r = status.get("robot", {})
            print(f"  Pi mode:     {a.get('mode', '?')}")
            print(f"  Pi enabled:  {a.get('enabled', '?')}")
            print(f"  Arduino:     {r.get('arduino_online', '?')}")
            print(f"  Motor out:   {r.get('motor_output', '?')}")

    print("\nLoading MediaPipe face model...")
    detector = FaceDetector(min_confidence=args.confidence)
    print("Opening Pi video stream...")
    cap = cv2.VideoCapture(args.pi_source)
    if not cap.isOpened():
        print(f"ERROR: cannot open {args.pi_source}")
        detector.close()
        return

    frame_idx = 0
    missing_streak = 0
    last_turn_at = 0.0
    pi_armed = False
    fps_time = time.monotonic()
    centering_direction: str | None = None

    print("\nStarting loop.")
    print("  J = center-left (keep turning left until face centered)")
    print("  L = center-right (keep turning right until face centered)")
    print("  ESC = quit\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            missing_streak += 1
            if missing_streak == 1 or missing_streak % 30 == 0:
                print(f"[frame {frame_idx}] stream read failed (streak={missing_streak})")
                if pi:
                    pi._log("stream_missing", {"frame": frame_idx, "streak": missing_streak})
            time.sleep(0.1)
            if missing_streak > 60:
                print("Stream lost for >60 frames, attempting reconnect...")
                cap.release()
                time.sleep(1.0)
                cap = cv2.VideoCapture(args.pi_source)
                missing_streak = 0
            continue
        missing_streak = 0
        frame_idx += 1

        face = detector.detect(frame)

        # ── Draw overlay ──────────────────────────────────────────────
        if face.detected:
            x1 = int(face.center_x - face.box_width / 2)
            y1 = int(face.center_y - face.box_height / 2)
            x2 = int(face.center_x + face.box_width / 2)
            y2 = int(face.center_y + face.box_height / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (int(face.center_x), int(face.center_y)), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"offX={face.offset_x:.0f}", (x1, max(22, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "NO FACE", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2)

        # Status bar
        now = time.monotonic()
        fps = 1.0 / max(0.001, now - fps_time)
        fps_time = now
        status_text = f"FPS:{fps:.0f} frame:{frame_idx}"
        if pi and pi_armed:
            status_text += " | Pi:ARMED"
        elif pi:
            status_text += " | Pi:OFF"
        if face.detected:
            status_text += f" | face:offX={face.offset_x:.0f}"
        cv2.putText(frame, status_text, (8, frame.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # ── Centering logic (J/L key latches direction until face centered) ──
        key = cv2.waitKey(1) & 0xFF
        if key == ord("j"):
            centering_direction = "LEFT_90"
            print(f"  [frame {frame_idx}] J → CENTER LEFT")
        elif key == ord("l"):
            centering_direction = "RIGHT_90"
            print(f"  [frame {frame_idx}] L → CENTER RIGHT")
        elif key == 27:
            break

        if centering_direction is not None and pi:
            # Face temporarily lost during rotation? Keep trying.
            if not face.detected:
                cv2.putText(frame, f"CENTERING {centering_direction} — waiting for face...", (12, 48),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
            elif face.offset_x is not None and abs(face.offset_x) <= args.deadband_px:
                centering_direction = None
                print(f"  [frame {frame_idx}] CENTERED (offX={face.offset_x:.0f})")
            elif now - last_turn_at > args.cooldown_seconds:
                if not pi_armed:
                    pi_armed = pi.toggle_m()
                if pi_armed:
                    ok = pi.send_turn(centering_direction)
                    if ok:
                        last_turn_at = now
                        print(f"  [frame {frame_idx}] {centering_direction} pulse (offX={face.offset_x:.0f})")
                    else:
                        last_turn_at = now

        # Status overlay
        cv2.putText(frame, "J=center-left  L=center-right  ESC=quit", (12, frame.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        if centering_direction:
            color = (0, 255, 0) if centering_direction == "LEFT_90" else (255, 0, 255)
            prefix = "WAITING" if not face.detected else "CENTERING"
            cv2.putText(frame, f"{prefix}: {centering_direction}  offX={face.offset_x:.0f}" if face.offset_x else f"{prefix}: {centering_direction}",
                        (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        cv2.imshow("Face -> Pi Turn (J/L center, ESC quit)", frame)

    cap.release()
    cv2.destroyAllWindows()
    detector.close()
    print(f"\nStopped. {frame_idx} frames processed. Log: {log_path}")


if __name__ == "__main__":
    main()
