"""Complete chip pipeline: YOLO localize → rectify → template match denomination."""
import argparse
import time
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

from chip_template_matcher import ChipTemplateMatcher
from chip_live_value import recognize_chip_value

MODEL = Path(__file__).resolve().parent / "chip_v2_model" / "best.pt"
TEMPLATES = Path(__file__).resolve().parent / "templates"
INPUT_DIR = Path(r"C:\!D\xwechat_files\wxid_7mj1gxlmr77812_d729\msg\file\2026-07\chip_v2\chip_v2")
OUTPUT_DIR = Path(__file__).resolve().parent / "output_value"

DENOM_NAMES = {1: "chip_1", 5: "chip_5", 10: "chip_10", 20: "chip_20"}
COLORS = {1: (230, 230, 230), 5: (90, 200, 255), 10: (80, 80, 255), 20: (255, 120, 80)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--templates", type=Path, default=TEMPLATES)
    parser.add_argument("--input", type=Path, default=INPUT_DIR)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    print("Loading YOLO localization model...")
    model = YOLO(str(args.model))
    model.predict(np.zeros((640, 640, 3), dtype=np.uint8), conf=args.conf,
                  imgsz=960, device="cpu", verbose=False)

    print("Loading denomination template matcher...")
    matcher = ChipTemplateMatcher(args.templates, minimum_score=0.55, minimum_margin=0.03,
                                  allowed_denominations=(10, 20))

    images = sorted(args.input.glob("*"))
    images = [p for p in images if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp", ".webp")]
    print(f"Processing {len(images)} images...\n")

    for i, img_path in enumerate(images):
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  [{i+1}/{len(images)}] {img_path.name} — skip")
            continue

        results = model.predict(frame, conf=args.conf, imgsz=960, device="cpu", verbose=False)[0]

        detections = []
        if results.boxes is not None:
            for xyxy, conf, cls_id in zip(results.boxes.xyxy.cpu().tolist(),
                                           results.boxes.conf.cpu().tolist(),
                                           results.boxes.cls.cpu().tolist()):
                x1, y1, x2, y2 = map(int, xyxy)
                detections.append({"bbox_xyxy": (x1, y1, x2, y2), "confidence": conf,
                                   "class": model.names[int(cls_id)]})

        print(f"  [{i+1}/{len(images)}] {img_path.name}: {len(detections)} chips", end="")

        for j, det in enumerate(detections):
            bbox = det["bbox_xyxy"]
            x1, y1, x2, y2 = bbox
            try:
                obs = recognize_chip_value(matcher, frame, bbox,
                                           minimum_minor_axis_px=30.0,
                                           minimum_aspect_ratio=0.30)
                denom = obs.denomination
                score = obs.score
                reason = obs.decision_reason or obs.rejection_reason or "?"
                ellipse_q = obs.ellipse_quality
            except Exception as e:
                denom, score, reason, ellipse_q = None, 0.0, str(e)[:60], None

            color = COLORS.get(denom, (70, 220, 70)) if denom else (100, 100, 100)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"${denom}" if denom else "?"
            label += f" {score:.2f}"
            cv2.putText(frame, label, (x1, max(20, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"{reason} e={ellipse_q}", (x1, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            print(f" ${denom}({score:.2f})" if denom else f" ?({reason[:20]})", end="")

        print()

        out_path = args.output / img_path.name
        cv2.imwrite(str(out_path), frame)

        if args.show:
            cv2.imshow("Chip Full Pipeline", frame)
            if cv2.waitKey(0) & 0xFF == 27:
                break

    cv2.destroyAllWindows()
    print(f"\nDone. Results in {args.output}")


if __name__ == "__main__":
    main()
