"""Publish DeskMate-Advance YuNet/SFace face position as control-neutral JSON.

The process runs on the PC, reads the Pi MJPEG stream through DeskMate's
OpenCVCamera adapter, and uses DeskMate's OpenCvFaceIdentityAdapter.  It never
imports robot control code and never sends motor commands.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DESKMATE_ROOT = PROJECT_ROOT / "subrepos" / "DeskMate-Advance"
DESKMATE_SRC = DESKMATE_ROOT / "src"
DESKMATE_FACE_CONFIG = (
    DESKMATE_ROOT / "configs" / "perception" / "face_identity_session.json"
)

if not DESKMATE_SRC.is_dir():
    raise RuntimeError(
        "DeskMate-Advance submodule is missing. Run "
        "`git submodule update --init --recursive` from the Robitics root."
    )
sys.path.insert(0, str(DESKMATE_SRC))

from poker_dealer.io.camera import (  # noqa: E402
    CameraConfig,
    CameraReadStatus,
    OpenCVCamera,
)
from poker_dealer.perception.identity import (  # noqa: E402
    DetectedFaceFeature,
    FaceFrameEvidence,
    FaceIdentityConfig,
    OpenCvFaceIdentityAdapter,
)


# DroidCam phone 192.168.137.157 is connected through the Windows client.
# While the client is active its HTTP /video endpoint reports "DroidCam is
# Busy"; OpenCV must read the registered local virtual camera instead.
DEFAULT_SOURCE = "1"
DEFAULT_LOCAL_BACKEND = "msmf"


def select_primary_feature(
    evidence: FaceFrameEvidence,
) -> DetectedFaceFeature | None:
    """Choose the largest usable face, with confidence as a stable tie-break."""

    return max(
        evidence.features,
        key=lambda feature: (
            feature.bbox_xywh[2] * feature.bbox_xywh[3],
            feature.detection_score,
        ),
        default=None,
    )


def face_payload(
    evidence: FaceFrameEvidence,
    *,
    frame_index: int,
    frame_width: int,
    frame_height: int,
    source: str,
    camera_reconnects: int = 0,
) -> dict[str, Any]:
    """Convert DeskMate evidence to the established PC/Pi face JSON contract."""

    primary = select_primary_feature(evidence)
    if primary is None:
        center_x = center_y = offset_x = offset_y = None
        box_width = box_height = 0
        score = 0.0
    else:
        x, y, box_width, box_height = primary.bbox_xywh
        center_x = x + box_width / 2
        center_y = y + box_height / 2
        offset_x = center_x - frame_width / 2
        offset_y = center_y - frame_height / 2
        score = primary.detection_score

    return {
        "time": datetime.now(timezone.utc).isoformat(),
        "frame": frame_index,
        "source": source,
        "model": "deskmate-opencv-yunet-sface",
        "detected": primary is not None,
        "detected_face_count": evidence.detected_face_count,
        "usable_face_count": len(evidence.features),
        "low_quality_face_count": evidence.low_quality_face_count,
        "center_x": center_x,
        "center_y": center_y,
        "offset_x": offset_x,
        "offset_y": offset_y,
        "offset_x_normalized": (
            None if offset_x is None else round(offset_x / (frame_width / 2), 4)
        ),
        "offset_y_normalized": (
            None if offset_y is None else round(offset_y / (frame_height / 2), 4)
        ),
        "box_width": box_width,
        "box_height": box_height,
        "score": round(score, 4),
        "frame_width": frame_width,
        "frame_height": frame_height,
        "processing_ms": round(evidence.inference_latency_ms, 2),
        "camera_reconnects": camera_reconnects,
        "error": "",
    }


def camera_config(
    source: str,
    *,
    local_backend: str = DEFAULT_LOCAL_BACKEND,
) -> CameraConfig:
    """Build DeskMate's bounded camera configuration for URL or local index."""

    if source.strip().isdigit():
        return CameraConfig(
            device_index=int(source),
            source_id="deskmate_face_camera",
            backend=local_backend,
            width=1280,
            height=720,
            fps=30.0,
        )
    return CameraConfig(
        stream_url=source,
        source_id="pi_mjpeg_face_camera",
        backend="auto",
        width=None,
        height=None,
        fps=None,
        disconnect_after_failures=3,
        open_timeout_ms=5000,
        read_timeout_ms=2000,
        reconnect_attempts=5,
        reconnect_backoff_ms=250,
    )


class DeskMateFacePositionPublisher:
    """Own the camera/model loop and expose only an immutable latest snapshot."""

    def __init__(
        self,
        source: str,
        *,
        local_backend: str = DEFAULT_LOCAL_BACKEND,
    ) -> None:
        self.source = source
        self.local_backend = local_backend
        self._lock = threading.Lock()
        self._latest: dict[str, Any] = {
            "time": datetime.now(timezone.utc).isoformat(),
            "source": source,
            "model": "deskmate-opencv-yunet-sface",
            "detected": False,
            "error": "starting",
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._latest)

    def start(self) -> None:
        threading.Thread(
            target=self.run,
            daemon=True,
            name="deskmate-face-position-publisher",
        ).start()

    def _set_error(self, error: str) -> None:
        with self._lock:
            self._latest = {
                "time": datetime.now(timezone.utc).isoformat(),
                "source": self.source,
                "model": "deskmate-opencv-yunet-sface",
                "detected": False,
                "score": 0.0,
                "error": error,
            }

    def run(self, *, max_frames: int | None = None) -> dict[str, Any]:
        """Run continuously, or process a bounded number of successful frames."""

        successful_frames = 0
        try:
            config = FaceIdentityConfig.from_json(DESKMATE_FACE_CONFIG)
            model = OpenCvFaceIdentityAdapter(config)
            with OpenCVCamera(
                camera_config(self.source, local_backend=self.local_backend)
            ) as camera:
                while max_frames is None or successful_frames < max_frames:
                    reading = camera.read()
                    if reading.status is not CameraReadStatus.OK or reading.frame is None:
                        self._set_error(
                            f"camera_{reading.status.value}:{reading.reason or 'unknown'}"
                        )
                        if (
                            max_frames is not None
                            and reading.status is CameraReadStatus.DISCONNECTED
                        ):
                            break
                        time.sleep(0.02)
                        continue
                    frame = reading.frame
                    evidence = model.analyze(frame)
                    successful_frames += 1
                    payload = face_payload(
                        evidence,
                        frame_index=frame.sequence_id,
                        frame_width=frame.width,
                        frame_height=frame.height,
                        source=self.source,
                        camera_reconnects=camera.network_reconnects,
                    )
                    with self._lock:
                        self._latest = payload
        except Exception as exc:
            self._set_error(f"publisher_error:{type(exc).__name__}:{exc}")
        return self.snapshot()


def make_handler(publisher: DeskMateFacePositionPublisher):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path not in {"/health", "/api/face/latest"}:
                self.send_error(404)
                return
            body = json.dumps(
                publisher.snapshot(), ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args: object) -> None:
            return

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(
        description="DeskMate YuNet/SFace PC face-position JSON publisher"
    )
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    parser.add_argument(
        "--backend",
        choices=("auto", "dshow", "msmf"),
        default=DEFAULT_LOCAL_BACKEND,
        help="OpenCV backend for a numeric local camera source",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5059)
    parser.add_argument(
        "--probe-frames",
        type=int,
        default=0,
        help="process N successful frames, print the last JSON, then exit",
    )
    args = parser.parse_args()
    if args.probe_frames < 0:
        parser.error("--probe-frames must be non-negative")

    publisher = DeskMateFacePositionPublisher(
        args.source,
        local_backend=args.backend,
    )
    if args.probe_frames:
        result = publisher.run(max_frames=args.probe_frames)
        print(json.dumps(result, ensure_ascii=False))
        raise SystemExit(0 if not result.get("error") else 2)

    publisher.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(publisher))
    print(
        f"DeskMate face JSON: http://{args.host}:{args.port}/api/face/latest\n"
        f"source={args.source}\n"
        "PC inference only; no robot, motor, Arduino, or route-control imports."
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
