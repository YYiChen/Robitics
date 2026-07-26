"""PC DroidCam preview for J/L face-centering turns controlled by the Pi."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time

import cv2


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from pi_service.robot_client import RobotClientConfig, RobotWebClient  # noqa: E402


WINDOW = "DroidCam face turn - J LEFT / L RIGHT / SPACE STOP / ESC EXIT"


def parse_source(value: str) -> int | str:
    text = value.strip()
    return int(text) if text.lstrip("-").isdigit() else text


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Use DroidCam to turn left/right until a face is centred.")
    parser.add_argument("--controller-url", default="http://100.80.46.54:5000")
    parser.add_argument(
        "--face-source",
        default="1",
        help="DroidCam virtual-camera index; this PC uses 1 while its USB webcam is 0, or pass http://PHONE:4747/video",
    )
    parser.add_argument("--send-fps", type=float, default=8.0)
    parser.add_argument("--face-min-size", type=int, default=60)
    parser.add_argument("--preview-only", action="store_true", help="show detection but never request a motor turn")
    parser.add_argument(
        "--cascade",
        type=Path,
        default=Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml",
    )
    return parser.parse_args()


class FaceDetector:
    def __init__(self, cascade_path: Path, minimum_size: int) -> None:
        self.minimum_size = minimum_size
        self.mediapipe_detector = None
        try:
            import mediapipe as mp

            self.mediapipe_detector = mp.solutions.face_detection.FaceDetection(
                model_selection=0,
                min_detection_confidence=0.45,
            )
        except (ImportError, AttributeError):
            pass
        self.classifier = cv2.CascadeClassifier(str(cascade_path))
        if self.mediapipe_detector is None and self.classifier.empty():
            raise RuntimeError(f"cannot load MediaPipe or Haar face detector: {cascade_path}")

    def detect(self, frame) -> tuple[tuple[int, int, int, int] | None, float | None]:
        height, frame_width = frame.shape[:2]
        if self.mediapipe_detector is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.mediapipe_detector.process(rgb)
            candidates = []
            for detection in result.detections or ():
                relative = detection.location_data.relative_bounding_box
                x = max(0, int(round(relative.xmin * frame_width)))
                y = max(0, int(round(relative.ymin * height)))
                width = min(frame_width - x, int(round(relative.width * frame_width)))
                box_height = min(height - y, int(round(relative.height * height)))
                if width >= self.minimum_size and box_height >= self.minimum_size:
                    candidates.append((x, y, width, box_height))
            if candidates:
                box = max(candidates, key=lambda item: item[2] * item[3])
                return box, float(box[0] + box[2] / 2.0)

        if self.classifier.empty():
            return None, None
        gray = cv2.equalizeHist(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        faces = self.classifier.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(self.minimum_size, self.minimum_size),
        )
        if len(faces) == 0:
            return None, None
        x, y, width, height = max(faces, key=lambda item: int(item[2]) * int(item[3]))
        return (int(x), int(y), int(width), int(height)), float(x + width / 2.0)

    def close(self) -> None:
        if self.mediapipe_detector is not None:
            self.mediapipe_detector.close()


def open_capture(source: int | str):
    if isinstance(source, int) and os.name == "nt":
        capture = cv2.VideoCapture(source, cv2.CAP_DSHOW)
    else:
        capture = cv2.VideoCapture(source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open DroidCam source: {source}")
    return capture


def annotate(frame, box, center_x, active: bool, side: str | None, status: str):
    output = frame.copy()
    height, width = output.shape[:2]
    middle = width // 2
    tolerance = int(round(width * 0.05))
    cv2.rectangle(output, (middle - tolerance, 0), (middle + tolerance, height - 1), (0, 190, 255), 2)
    cv2.line(output, (middle, 0), (middle, height - 1), (255, 255, 0), 1)
    if box is not None:
        x, y, face_width, face_height = box
        cv2.rectangle(output, (x, y), (x + face_width, y + face_height), (0, 255, 0), 2)
        cv2.circle(output, (int(center_x), y + face_height // 2), 5, (0, 255, 0), -1)

    button_top = max(0, height - 58)
    left_rect = (15, button_top, min(235, width // 2 - 8), height - 12)
    right_rect = (max(width // 2 + 8, width - 235), button_top, width - 15, height - 12)
    for rect, label, selected in (
        (left_rect, "J  LEFT FACE TURN", active and side == "LEFT"),
        (right_rect, "L  RIGHT FACE TURN", active and side == "RIGHT"),
    ):
        x1, y1, x2, y2 = rect
        cv2.rectangle(output, (x1, y1), (x2, y2), (40, 150, 60) if selected else (60, 80, 110), -1)
        cv2.putText(output, label, (x1 + 10, y1 + 29), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)

    face_text = "face=none" if center_x is None else f"face_x={center_x:.1f}/{width}"
    lines = (
        f"{'ACTIVE ' + str(side) if active else 'IDLE'}  {face_text}",
        status,
        "J/L start or change direction | SPACE stop | ESC exit",
    )
    for index, text in enumerate(lines):
        cv2.putText(output, text, (14, 26 + index * 24), cv2.FONT_HERSHEY_SIMPLEX, .53, (255, 255, 255), 2)
    return output, left_rect, right_rect


def main() -> int:
    args = parse_args()
    source = parse_source(args.face_source)
    detector = FaceDetector(args.cascade, args.face_min_size)
    capture = open_capture(source)
    client = None if args.preview_only else RobotWebClient(RobotClientConfig(args.controller_url, timeout_seconds=1.2))
    if client is not None:
        status = client.require_arduino_online()
        if status.get("autonomous", {}).get("mode") != "end_line_turn_adaptor":
            raise RuntimeError("Pi must run route mode end_line_turn_adaptor before J/L face turns")

    pending = {"action": None}
    latest_button_rects = {"LEFT": (0, 0, 0, 0), "RIGHT": (0, 0, 0, 0)}

    def on_mouse(event, x, y, _flags, _data):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for side, (x1, y1, x2, y2) in latest_button_rects.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                pending["action"] = side

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW, on_mouse)
    active, side = False, None
    status_text = f"DroidCam source={source} ready; Edge J/L control enabled"
    last_send_at = 0.0
    interval = 1.0 / max(1.0, args.send_fps)

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("DroidCam stream ended")
            box, center_x = detector.detect(frame)
            visual, left_rect, right_rect = annotate(frame, box, center_x, active, side, status_text)
            latest_button_rects["LEFT"], latest_button_rects["RIGHT"] = left_rect, right_rect
            cv2.imshow(WINDOW, visual)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == ord(" "):
                pending["action"] = "STOP"
            elif key in (ord("j"), ord("J")):
                pending["action"] = "LEFT"
            elif key in (ord("l"), ord("L")):
                pending["action"] = "RIGHT"

            action = pending.pop("action", None)
            pending["action"] = None
            if action == "STOP":
                if client is not None and active:
                    client.cancel_face_turn()
                active, side, status_text = False, None, "face turn cancelled; motors stopped"
            elif action in {"LEFT", "RIGHT"}:
                side = action
                if client is not None:
                    response = client.start_face_turn(side)
                    active = bool(response.get("face_turn_active"))
                else:
                    active = True
                status_text = f"{'J left' if side == 'LEFT' else 'L right'} face search started"
                last_send_at = 0.0

            now = time.monotonic()
            if client is not None and now - last_send_at >= interval:
                last_send_at = now
                response = client.send_face_observation(
                    found=center_x is not None,
                    frame_width=frame.shape[1],
                    center_x=center_x,
                )
                active = bool(response.get("face_turn_active"))
                side = response.get("face_search_side") if active else None
                status_text = str(response.get("detail") or response.get("state") or "face observation sent")
            elif client is None and active and center_x is not None:
                offset = (center_x - frame.shape[1] / 2.0) / max(1.0, frame.shape[1] / 2.0)
                status_text = f"preview offset={offset:+.3f}"
    finally:
        if client is not None:
            try:
                client.cancel_face_turn()
            except RuntimeError:
                pass
        detector.close()
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
