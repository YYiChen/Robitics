"""Run YOLO chip detector on a folder of images, save annotated output."""
import argparse
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO

MODEL = Path(__file__).resolve().parent / "chip_v2_model" / "best.pt"
INPUT_DIR = Path(r"C:\!D\xwechat_files\wxid_7mj1gxlmr77812_d729\msg\file\2026-07\chip_v2\chip_v2")
OUTPUT_DIR = Path(__file__).resolve().parent / "output"

COLORS = [(70, 220, 70), (230, 230, 230), (90, 200, 255), (80, 80, 255), (255, 120, 80)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--input", type=Path, default=INPUT_DIR)
    parser.add_argument("--output", type=Path, default=OUTPUT_DIR)
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--show", action="store_true", help="also show each result in a window")
    args = parser.parse_args()

    if not args.model.is_file():
        raise SystemExit(f"Model not found: {args.model}")
    if not args.input.is_dir():
        raise SystemExit(f"Input dir not found: {args.input}")

    args.output.mkdir(parents=True, exist_ok=True)

    print(f"Loading model: {args.model}")
    model = YOLO(str(args.model))

    # Warm up
    model.predict(np.zeros((640, 640, 3), dtype=np.uint8), conf=args.conf,
                  imgsz=args.imgsz, device="cpu", verbose=False)

    images = sorted(args.input.glob("*"))
    images = [p for p in images if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp", ".webp")]
    print(f"Found {len(images)} images in {args.input}")

    for i, img_path in enumerate(images):
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  [{i+1}/{len(images)}] {img_path.name} — cannot read, skipping")
            continue

        results = model.predict(frame, conf=args.conf, imgsz=args.imgsz,
                                device="cpu", verbose=False)[0]

        chips_found = 0
        if results.boxes is not None:
            for xyxy, conf, cls_id in zip(results.boxes.xyxy.cpu().tolist(),
                                           results.boxes.conf.cpu().tolist(),
                                           results.boxes.cls.cpu().tolist()):
                x1, y1, x2, y2 = map(int, xyxy)
                color = COLORS[int(cls_id) % len(COLORS)]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{model.names[int(cls_id)]} {conf:.2f}"
                cv2.putText(frame, label, (x1, max(20, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                chips_found += 1

        out_path = args.output / img_path.name
        cv2.imwrite(str(out_path), frame)
        print(f"  [{i+1}/{len(images)}] {img_path.name} → {chips_found} chips → {out_path.name}")

        if args.show:
            cv2.imshow("Chip Detection", frame)
            if cv2.waitKey(0) & 0xFF == 27:
                break

    cv2.destroyAllWindows()
    print(f"\nDone. Annotated images saved to {args.output}")


if __name__ == "__main__":
    main()
