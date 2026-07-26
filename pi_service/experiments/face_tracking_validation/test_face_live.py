"""Live test: MediaPipe face detection from phone or Pi camera MJPEG stream."""
import argparse
import cv2
import time
from face_detector import FaceDetector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://100.80.46.54:5000/video_feed",
                        help="MJPEG stream URL (default: Pi camera)")
    args = parser.parse_args()

    print(f"Connecting to {args.url} ...")
    source = int(args.url) if args.url.isdigit() else args.url
    cap = cv2.VideoCapture(source)
    print("Loading MediaPipe face model (first run downloads ~200KB)...")
    detector = FaceDetector()

    frame_idx = 0
    fps_time = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            frame_idx += 1
            if frame_idx % 30 == 0:
                print(f"  frame read failed ({frame_idx} times), reconnecting...")
                cap.release()
                time.sleep(0.5)
                cap = cv2.VideoCapture(source)
            continue

        face = detector.detect(frame)

        if face.detected:
            x1 = int(face.center_x - face.box_width / 2)
            y1 = int(face.center_y - face.box_height / 2)
            x2 = int(face.center_x + face.box_width / 2)
            y2 = int(face.center_y + face.box_height / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (int(face.center_x), int(face.center_y)), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"X:{face.center_x:.0f} offX:{face.offset_x:.0f} ({face.score*100:.0f}%)",
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        now = time.time()
        fps = 1.0 / max(0.001, now - fps_time)
        fps_time = now
        cv2.putText(frame, f"FPS:{fps:.0f}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)

        cv2.imshow("MediaPipe Face", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
        frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()
    detector.close()
    print(f"Done. Processed {frame_idx} frames.")


if __name__ == "__main__":
    main()
