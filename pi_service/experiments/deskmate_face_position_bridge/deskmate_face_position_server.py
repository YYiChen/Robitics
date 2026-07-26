"""Run the single-process DeskMate face-control stack on the Windows PC.

One camera owner feeds DeskMate inference and the 5059 annotated relay.  An
embedded bridge reads the same in-memory observation and only renews or stops
an already-started Pi face turn through the versioned robotics-v1 API.  It
never starts a turn and never imports Pi motor-control code.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any
from urllib.parse import urlparse
from urllib.request import urlopen

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DESKMATE_ROOT = PROJECT_ROOT / "subrepos" / "DeskMate-Advance"
DESKMATE_SRC = DESKMATE_ROOT / "src"
DESKMATE_FACE_CONFIG = (
    DESKMATE_ROOT / "configs" / "perception" / "face_identity_session.json"
)
FACE_TRACKING_DIR = (
    PROJECT_ROOT / "pi_service" / "experiments" / "face_tracking_validation"
)
LEGACY_BRIDGE_SCRIPT = FACE_TRACKING_DIR / "face_turn_web_bridge.py"

if not DESKMATE_SRC.is_dir():
    raise RuntimeError(
        "DeskMate-Advance submodule is missing. Run "
        "`git submodule update --init --recursive` from the Robitics root."
    )
sys.path.insert(0, str(DESKMATE_SRC))
sys.path.insert(0, str(FACE_TRACKING_DIR))

from face_turn_web_bridge import FaceTurnBridge  # noqa: E402
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


# The direct DroidCam MJPEG source is preferred. If the Windows DroidCam client
# owns the phone, this endpoint returns a "DroidCam is Busy" HTML page instead
# of video; close that client or explicitly select its local virtual camera.
DEFAULT_SOURCE = "http://100.93.97.117:4747/video"
DEFAULT_LOCAL_BACKEND = "msmf"
DEFAULT_DETECTOR_SCORE_THRESHOLD = 0.75
DEFAULT_MINIMUM_FACE_SIZE_PX = 40
DEFAULT_PREVIEW_FPS = 10.0
DEFAULT_PREVIEW_JPEG_QUALITY = 78
DEFAULT_CENTER_DEADBAND_NORMALIZED = 0.30
DEFAULT_SERVER_PORT = 5059
DEFAULT_CAMERA_RETRY_SECONDS = 1.0
DEFAULT_PI_URL = "http://100.80.46.54:5000"
DEFAULT_BRIDGE_HEARTBEAT_SECONDS = .18
DEFAULT_BRIDGE_MINIMUM_SCORE = .5
DEFAULT_BRIDGE_MAX_AGE_MS = 450


def _configured_port(cmdline: list[str]) -> int:
    """Return this server command's selected port without running argparse."""

    for index, argument in enumerate(cmdline):
        if argument == "--port" and index + 1 < len(cmdline):
            try:
                return int(cmdline[index + 1])
            except ValueError:
                return DEFAULT_SERVER_PORT
        if argument.startswith("--port="):
            try:
                return int(argument.partition("=")[2])
            except ValueError:
                return DEFAULT_SERVER_PORT
    return DEFAULT_SERVER_PORT


def _command_runs_script(
    cmdline: list[str],
    cwd: str | None,
    expected_script: Path,
) -> bool:
    """Match only a process executing one exact checked-out script."""

    expected = expected_script.resolve()
    base = Path(cwd).resolve() if cwd else None
    for argument in cmdline[1:]:
        if not argument.lower().endswith(expected.name.lower()):
            continue
        candidate = Path(argument)
        if not candidate.is_absolute():
            if base is None:
                continue
            candidate = base / candidate
        try:
            if candidate.resolve() == expected:
                return True
        except OSError:
            continue
    return False


def _command_runs_this_server(cmdline: list[str], cwd: str | None) -> bool:
    return _command_runs_script(cmdline, cwd, Path(__file__))


def _is_python_runtime(process_name: str) -> bool:
    """Exclude launchers such as py.exe while accepting real Python runtimes."""

    return Path(process_name).stem.lower().startswith("python")


def replace_existing_server_instances(
    port: int,
    *,
    wait_seconds: float = 3.0,
) -> list[int]:
    """Stop only stale copies of this script configured for the same port."""

    try:
        import psutil
    except ImportError as exc:
        raise RuntimeError(
            "Automatic single-instance replacement requires psutil. "
            "Install it with `py -3 -m pip install psutil`, or start with "
            "`--no-replace-existing`."
        ) from exc

    current_pid = os.getpid()
    matches = []
    for process in psutil.process_iter(["pid", "name", "cmdline"]):
        if process.pid == current_pid:
            continue
        try:
            process_name = process.name()
            cmdline = process.cmdline()
            cwd = process.cwd()
        except (psutil.AccessDenied, psutil.NoSuchProcess, OSError):
            continue
        if (
            cmdline
            and _is_python_runtime(process_name)
            and _configured_port(cmdline) == port
            and _command_runs_this_server(cmdline, cwd)
        ):
            matches.append(process)

    for process in matches:
        try:
            process.terminate()
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    _gone, alive = psutil.wait_procs(matches, timeout=max(0.1, wait_seconds))
    for process in alive:
        try:
            process.kill()
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    if alive:
        psutil.wait_procs(alive, timeout=max(0.1, wait_seconds))
    return [process.pid for process in matches]


def replace_legacy_bridge_instances(
    *,
    wait_seconds: float = 3.0,
) -> list[int]:
    """Stop standalone bridges superseded by this embedded single process."""

    try:
        import psutil
    except ImportError as exc:
        raise RuntimeError(
            "Automatic legacy bridge cleanup requires psutil. "
            "Install it with `py -3 -m pip install psutil`."
        ) from exc

    matches = []
    for process in psutil.process_iter(["pid", "name", "cmdline"]):
        if process.pid == os.getpid():
            continue
        try:
            process_name = process.name()
            cmdline = process.cmdline()
            cwd = process.cwd()
        except (psutil.AccessDenied, psutil.NoSuchProcess, OSError):
            continue
        if (
            cmdline
            and _is_python_runtime(process_name)
            and _command_runs_script(cmdline, cwd, LEGACY_BRIDGE_SCRIPT)
        ):
            matches.append(process)

    for process in matches:
        try:
            process.terminate()
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    _gone, alive = psutil.wait_procs(matches, timeout=max(0.1, wait_seconds))
    for process in alive:
        try:
            process.kill()
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    if alive:
        psutil.wait_procs(alive, timeout=max(0.1, wait_seconds))
    return [process.pid for process in matches]


def diagnose_network_source(source: str) -> str | None:
    """Explain common DroidCam HTML responses after OpenCV failed to open."""

    parsed = urlparse(source)
    if parsed.scheme not in {"http", "https"}:
        return None
    try:
        with urlopen(source, timeout=2.0) as response:
            content_type = str(response.headers.get("Content-Type", "")).lower()
            if (
                content_type.startswith("multipart/")
                or content_type.startswith("video/")
                or content_type.startswith("image/")
            ):
                return None
            sample = response.read(4096).decode("utf-8", "replace").lower()
    except Exception:
        return None
    if "droidcam is busy" in sample or "droidcam_busy" in sample:
        return (
            "camera_source_busy:droidcam_single_client_in_use; close every "
            "browser/client viewing the phone /video URL and use "
            "http://127.0.0.1:5059/preview_feed instead"
        )
    return f"camera_source_not_video:content_type={content_type or 'unknown'}"


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


def annotate_face_preview(
    image: np.ndarray,
    evidence: FaceFrameEvidence,
    payload: dict[str, Any],
    *,
    deadband_normalized: float = DEFAULT_CENTER_DEADBAND_NORMALIZED,
) -> np.ndarray:
    """Draw usable face boxes and the exact PC bridge centre gate."""

    annotated = np.asarray(image).copy()
    height, width = annotated.shape[:2]
    center_x = width // 2
    gate_half_width = int(round(width * deadband_normalized / 2))
    gate_left = max(0, center_x - gate_half_width)
    gate_right = min(width - 1, center_x + gate_half_width)

    cv2.line(annotated, (center_x, 0), (center_x, height - 1), (255, 220, 70), 2)
    cv2.line(
        annotated, (gate_left, 0), (gate_left, height - 1), (60, 200, 255), 2
    )
    cv2.line(
        annotated, (gate_right, 0), (gate_right, height - 1), (60, 200, 255), 2
    )

    primary = select_primary_feature(evidence)
    for feature in evidence.features:
        x, y, box_width, box_height = feature.bbox_xywh
        is_primary = feature is primary
        colour = (70, 230, 90) if is_primary else (80, 180, 255)
        thickness = 3 if is_primary else 2
        cv2.rectangle(
            annotated,
            (x, y),
            (min(width - 1, x + box_width), min(height - 1, y + box_height)),
            colour,
            thickness,
        )
        cv2.putText(
            annotated,
            f"{feature.detection_score:.2f}",
            (x, max(22, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            colour,
            2,
            cv2.LINE_AA,
        )

    detected = bool(payload.get("detected"))
    offset = payload.get("offset_x_normalized")
    centred = (
        detected
        and offset is not None
        and abs(float(offset)) <= deadband_normalized
    )
    state = "CENTERED" if centred else ("FACE" if detected else "NO FACE")
    state_colour = (
        (70, 230, 90)
        if centred
        else ((60, 200, 255) if detected else (70, 70, 240))
    )
    status = (
        f"{state}  raw={evidence.detected_face_count} "
        f"usable={len(evidence.features)}"
    )
    if offset is not None:
        status += f"  offset={float(offset):+.3f}"
    cv2.rectangle(annotated, (0, 0), (width - 1, 42), (12, 18, 26), -1)
    cv2.putText(
        annotated,
        status,
        (12, 29),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        state_colour,
        2,
        cv2.LINE_AA,
    )
    return annotated


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


def face_identity_config(
    *,
    detector_score_threshold: float = DEFAULT_DETECTOR_SCORE_THRESHOLD,
    minimum_face_size_px: int = DEFAULT_MINIMUM_FACE_SIZE_PX,
) -> FaceIdentityConfig:
    """Apply car-following detection thresholds without editing the submodule."""

    base = FaceIdentityConfig.from_json(DESKMATE_FACE_CONFIG)
    detector_options = dict(base.detector_options)
    detector_options["score_threshold"] = detector_score_threshold
    detector_options["minimum_face_size_px"] = minimum_face_size_px
    return replace(base, detector_options=detector_options)


class DeskMateFacePositionPublisher:
    """Own the camera/model loop and expose only an immutable latest snapshot."""

    def __init__(
        self,
        source: str,
        *,
        local_backend: str = DEFAULT_LOCAL_BACKEND,
        preview_fps: float = DEFAULT_PREVIEW_FPS,
        detector_score_threshold: float = DEFAULT_DETECTOR_SCORE_THRESHOLD,
        minimum_face_size_px: int = DEFAULT_MINIMUM_FACE_SIZE_PX,
    ) -> None:
        self.source = source
        self.local_backend = local_backend
        self.preview_fps = preview_fps
        self.detector_score_threshold = detector_score_threshold
        self.minimum_face_size_px = minimum_face_size_px
        self._condition = threading.Condition()
        self._latest: dict[str, Any] = {
            "time": datetime.now(timezone.utc).isoformat(),
            "source": source,
            "model": "deskmate-opencv-yunet-sface",
            "detected": False,
            "error": "starting",
        }
        self._bridge_latest: dict[str, Any] = {
            "enabled": False,
            "action": "not_started",
            "error": "",
        }
        self._preview_input: tuple[np.ndarray, FaceFrameEvidence, dict[str, Any]] | None = None
        self._preview_jpeg: bytes | None = None
        self._preview_sequence = 0
        self._preview_started = False

    def snapshot(self) -> dict[str, Any]:
        with self._condition:
            snapshot = dict(self._latest)
            snapshot["bridge"] = dict(self._bridge_latest)
            return snapshot

    def set_bridge_status(self, record: dict[str, Any]) -> None:
        with self._condition:
            self._bridge_latest = {"enabled": True, **record}

    def latest_preview(self) -> tuple[bytes | None, int]:
        with self._condition:
            return self._preview_jpeg, self._preview_sequence

    def wait_for_preview(
        self,
        sequence: int,
        *,
        timeout: float = 1.0,
    ) -> tuple[bytes | None, int]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._preview_sequence != sequence,
                timeout=timeout,
            )
            return self._preview_jpeg, self._preview_sequence

    def start(self) -> None:
        if not self._preview_started:
            self._preview_started = True
            threading.Thread(
                target=self._preview_encoder,
                daemon=True,
                name="deskmate-face-preview-encoder",
            ).start()
        threading.Thread(
            target=self._run_forever,
            daemon=True,
            name="deskmate-face-position-publisher",
        ).start()

    def _run_forever(self) -> None:
        """Recover when a network camera is busy or temporarily unavailable."""

        while True:
            self.run()
            time.sleep(DEFAULT_CAMERA_RETRY_SECONDS)

    def _set_error(self, error: str) -> None:
        with self._condition:
            self._latest = {
                "time": datetime.now(timezone.utc).isoformat(),
                "source": self.source,
                "model": "deskmate-opencv-yunet-sface",
                "detected": False,
                "score": 0.0,
                "error": error,
            }

    def _queue_preview(
        self,
        image: np.ndarray,
        evidence: FaceFrameEvidence,
        payload: dict[str, Any],
    ) -> None:
        with self._condition:
            self._preview_input = (image.copy(), evidence, dict(payload))
            self._condition.notify_all()

    def _preview_encoder(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._preview_input is not None)
                queued = self._preview_input
                self._preview_input = None
            if queued is None:
                continue
            image, evidence, payload = queued
            annotated = annotate_face_preview(image, evidence, payload)
            ok, encoded = cv2.imencode(
                ".jpg",
                annotated,
                [cv2.IMWRITE_JPEG_QUALITY, DEFAULT_PREVIEW_JPEG_QUALITY],
            )
            if not ok:
                continue
            with self._condition:
                self._preview_jpeg = encoded.tobytes()
                self._preview_sequence += 1
                self._condition.notify_all()

    def run(self, *, max_frames: int | None = None) -> dict[str, Any]:
        """Run continuously, or process a bounded number of successful frames."""

        successful_frames = 0
        next_preview_at = 0.0
        try:
            config = face_identity_config(
                detector_score_threshold=self.detector_score_threshold,
                minimum_face_size_px=self.minimum_face_size_px,
            )
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
                    payload["detector_score_threshold"] = float(
                        config.detector_options["score_threshold"]
                    )
                    payload["minimum_face_size_px"] = int(
                        config.detector_options["minimum_face_size_px"]
                    )
                    with self._condition:
                        self._latest = payload
                    now = time.monotonic()
                    if (
                        self._preview_started
                        and self.preview_fps > 0
                        and now >= next_preview_at
                    ):
                        self._queue_preview(frame.image, evidence, payload)
                        next_preview_at = now + 1.0 / self.preview_fps
        except Exception as exc:
            diagnosis = diagnose_network_source(self.source)
            self._set_error(
                diagnosis
                or f"publisher_error:{type(exc).__name__}:{exc}"
            )
        return self.snapshot()


def make_handler(publisher: DeskMateFacePositionPublisher):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/preview_feed":
                self.send_response(200)
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                sequence = 0
                try:
                    while True:
                        jpeg, sequence = publisher.wait_for_preview(sequence)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            if path == "/preview.jpg":
                jpeg, _sequence = publisher.latest_preview()
                if jpeg is None:
                    self.send_error(503, "preview frame is not ready")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(jpeg)
                return
            if path not in {"/health", "/api/face/latest"}:
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


def start_embedded_face_bridge(
    publisher: DeskMateFacePositionPublisher,
    *,
    pi_url: str,
    heartbeat_seconds: float,
    minimum_score: float,
    deadband_normalized: float,
    max_age_ms: int,
) -> Path:
    """Run the Pi lease bridge against the publisher's in-memory snapshot."""

    log_dir = FACE_TRACKING_DIR / "runtime_logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / (
        "face_control_stack_"
        f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.jsonl"
    )
    bridge = FaceTurnBridge(
        face_provider=publisher.snapshot,
        pi_url=pi_url,
        heartbeat_seconds=heartbeat_seconds,
        minimum_score=minimum_score,
        deadband_normalized=deadband_normalized,
        max_age_ms=max_age_ms,
    )

    def worker() -> None:
        with log_path.open("a", encoding="utf-8") as log:
            bridge.run_forever(
                log,
                record_callback=publisher.set_bridge_status,
            )

    threading.Thread(
        target=worker,
        daemon=True,
        name="deskmate-embedded-face-turn-bridge",
    ).start()
    return log_path


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
    parser.add_argument("--port", type=int, default=DEFAULT_SERVER_PORT)
    parser.add_argument("--pi-url", default=DEFAULT_PI_URL)
    parser.add_argument(
        "--disable-pi-bridge",
        action="store_true",
        help="publish face data only; do not heartbeat or stop Pi face turns",
    )
    parser.add_argument(
        "--bridge-heartbeat-seconds",
        type=float,
        default=DEFAULT_BRIDGE_HEARTBEAT_SECONDS,
    )
    parser.add_argument(
        "--bridge-minimum-score",
        type=float,
        default=DEFAULT_BRIDGE_MINIMUM_SCORE,
    )
    parser.add_argument(
        "--bridge-deadband-normalized",
        type=float,
        default=DEFAULT_CENTER_DEADBAND_NORMALIZED,
    )
    parser.add_argument(
        "--bridge-max-age-ms",
        type=int,
        default=DEFAULT_BRIDGE_MAX_AGE_MS,
    )
    parser.add_argument(
        "--no-replace-existing",
        action="store_true",
        help=(
            "do not stop an older copy of this exact server using the same port"
        ),
    )
    parser.add_argument(
        "--preview-fps",
        type=float,
        default=DEFAULT_PREVIEW_FPS,
        help="maximum annotated preview FPS; inference remains unthrottled",
    )
    parser.add_argument(
        "--detector-score-threshold",
        type=float,
        default=DEFAULT_DETECTOR_SCORE_THRESHOLD,
        help="YuNet detection threshold for this car-following experiment",
    )
    parser.add_argument(
        "--minimum-face-size-px",
        type=int,
        default=DEFAULT_MINIMUM_FACE_SIZE_PX,
        help="minimum usable face-box side length",
    )
    parser.add_argument(
        "--probe-frames",
        type=int,
        default=0,
        help="process N successful frames, print the last JSON, then exit",
    )
    args = parser.parse_args()
    if args.probe_frames < 0:
        parser.error("--probe-frames must be non-negative")
    if not 0 < args.preview_fps <= 30:
        parser.error("--preview-fps must be in (0, 30]")
    if not 0 < args.detector_score_threshold <= 1:
        parser.error("--detector-score-threshold must be in (0, 1]")
    if not 16 <= args.minimum_face_size_px <= 512:
        parser.error("--minimum-face-size-px must be in [16, 512]")
    if not .05 <= args.bridge_heartbeat_seconds <= 2:
        parser.error("--bridge-heartbeat-seconds must be in [0.05, 2]")
    if not 0 < args.bridge_minimum_score <= 1:
        parser.error("--bridge-minimum-score must be in (0, 1]")
    if not 0 < args.bridge_deadband_normalized <= 1:
        parser.error("--bridge-deadband-normalized must be in (0, 1]")
    if not 100 <= args.bridge_max_age_ms <= 5000:
        parser.error("--bridge-max-age-ms must be in [100, 5000]")

    publisher = DeskMateFacePositionPublisher(
        args.source,
        local_backend=args.backend,
        preview_fps=args.preview_fps,
        detector_score_threshold=args.detector_score_threshold,
        minimum_face_size_px=args.minimum_face_size_px,
    )
    if args.probe_frames:
        result = publisher.run(max_frames=args.probe_frames)
        print(json.dumps(result, ensure_ascii=False))
        raise SystemExit(0 if not result.get("error") else 2)

    if not args.no_replace_existing:
        replaced_pids = replace_existing_server_instances(args.port)
        if replaced_pids:
            print(
                "Replaced previous DeskMate face server process(es): "
                + ", ".join(str(pid) for pid in replaced_pids)
            )
        if not args.disable_pi_bridge:
            legacy_bridge_pids = replace_legacy_bridge_instances()
            if legacy_bridge_pids:
                print(
                    "Replaced standalone face bridge process(es): "
                    + ", ".join(str(pid) for pid in legacy_bridge_pids)
                )

    publisher.start()
    bridge_log_path = None
    if not args.disable_pi_bridge:
        bridge_log_path = start_embedded_face_bridge(
            publisher,
            pi_url=args.pi_url,
            heartbeat_seconds=args.bridge_heartbeat_seconds,
            minimum_score=args.bridge_minimum_score,
            deadband_normalized=args.bridge_deadband_normalized,
            max_age_ms=args.bridge_max_age_ms,
        )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(publisher))
    server.daemon_threads = True
    print(
        f"DeskMate face JSON: http://{args.host}:{args.port}/api/face/latest\n"
        f"Annotated preview: http://{args.host}:{args.port}/preview_feed\n"
        f"source={args.source}\n"
        + (
            f"Embedded Pi bridge: {args.pi_url} "
            f"(log={bridge_log_path})\n"
            if bridge_log_path is not None
            else "Embedded Pi bridge: disabled\n"
        )
        + "One camera owner; this process never starts a turn or imports motor code."
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
