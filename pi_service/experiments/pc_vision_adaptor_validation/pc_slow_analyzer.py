#!/usr/bin/env python3
"""Desktop companion: fetch Pi frames and post only validated visual events.

It is intentionally a first integration probe.  It runs the existing costly
green/white analyzer on the PC, records JSONL, and leaves wheel PWM entirely
to the Raspberry Pi adapter.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np

HERE = Path(__file__).resolve()
GREEN_EXPERIMENT = HERE.parents[1] / "i_shape_green_white_turnaround_validation"
if str(GREEN_EXPERIMENT) not in sys.path:
    sys.path.insert(0, str(GREEN_EXPERIMENT))
from green_white_scanline_i_logic import GreenWhiteHybridScanlineAnalyzer  # noqa: E402
from red_band_planner import TwoRedBandPlanner  # noqa: E402


def _pi_centers(header: str) -> list[tuple[int, float, int]]:
    centers = []
    for item in header.split(";"):
        try:
            y, x, width = item.split(",")
            centers.append((int(y), float(x), int(width)))
        except ValueError:
            continue
    return centers


def render_complete_overlay(frame, analyzer, component_mask, evidence, red, event: str, pi_centers, pi_confidence: float):
    """Render PC masks/geometry and Pi quick-scan data on one JPEG."""
    output = frame.copy()
    candidate_mask = analyzer.tape_candidate_mask
    if candidate_mask is not None and candidate_mask.any():
        layer = output.copy(); layer[candidate_mask > 0] = (90, 210, 255)
        output = cv2.addWeighted(output, .72, layer, .28, 0)
    if component_mask is not None and component_mask.any():
        layer = output.copy(); layer[component_mask > 0] = (0, 220, 255)
        output = cv2.addWeighted(output, .62, layer, .38, 0)
    field = analyzer.course_field_mask
    if field is not None and field.any():
        contours, _ = cv2.findContours(field, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, (0, 0, 255), 2, cv2.LINE_AA)
    red_mask = analyzer.red_marker_mask
    if red_mask is not None and red_mask.any():
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, (0, 0, 255), 3, cv2.LINE_AA)
    if analyzer.tape_fit_line is not None:
        cv2.line(output, analyzer.tape_fit_line[0], analyzer.tape_fit_line[1], (0, 0, 0), 3, cv2.LINE_AA)
    for y, x, _width in evidence.line_centers:
        cv2.circle(output, (int(x), y), 5, (0, 255, 0), -1)
    for y, x, _width in pi_centers:
        cv2.drawMarker(output, (int(x), y), (255, 255, 0), cv2.MARKER_CROSS, 12, 2)
    if evidence.endpoint_y is not None:
        cv2.line(output, (0, evidence.endpoint_y), (output.shape[1]-1, evidence.endpoint_y), (0, 165, 255), 2)
    if evidence.junction_detected and evidence.junction_y is not None:
        cv2.line(output, (0, evidence.junction_y), (output.shape[1]-1, evidence.junction_y), (255, 0, 255), 1)
    cv2.rectangle(output, (8, 8), (990, 154), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, "PC FULL ANALYSIS -> PI SAFE MOTOR ADAPTOR", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, .63, (0, 220, 0), 2)
    cv2.putText(output, f"PC EVENT: {event}  RED: {red.phase} layers={red.layer_count} turn=({red.turn_y},{red.turn_bottom_y})", (16, 60), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 165, 255), 1)
    cv2.putText(output, f"PC: conf={evidence.confidence:.2f} line={evidence.line_center_x} bar={evidence.endpoint_detected}@{evidence.endpoint_y} junction={evidence.junction_detected}@{evidence.junction_y}", (16, 84), cv2.FONT_HERSHEY_SIMPLEX, .40, (255,255,255), 1)
    cv2.putText(output, f"PI FAST cyan-x: conf={pi_confidence:.2f} rows={len(pi_centers)} | PC green-dot: scanline", (16, 108), cv2.FONT_HERSHEY_SIMPLEX, .40, (255,255,0), 1)
    cv2.putText(output, "yellow=tape / selected route; black=fit; red=green-field + red tape; orange=bar; purple=junction", (16, 132), cv2.FONT_HERSHEY_SIMPLEX, .38, (0,255,255), 1)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="PC slow visual analyser; never sends PWM")
    parser.add_argument("--pi-url", default="http://100.80.46.54:5000")
    parser.add_argument("--token", default="")
    parser.add_argument("--log", type=Path, default=Path("runtime_logs/pc_slow_analyzer.jsonl"))
    parser.add_argument("--max-frames", type=int, default=0, help="process this many frames then exit; 0 means run continuously")
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    analyzer, planner, last_seq, processed = GreenWhiteHybridScanlineAnalyzer(), TwoRedBandPlanner(), -1, 0
    while True:
        try:
            with urlopen(args.pi_url.rstrip("/") + "/api/vision-adaptor/frame", timeout=2) as response:
                jpeg = response.read(); headers = response.headers
            frame_seq = int(headers.get("X-Vision-Frame-Seq", "-1"))
            captured_at_ms = int(headers.get("X-Vision-Captured-At-Ms", "0"))
            pi_centers = _pi_centers(headers.get("X-Vision-Pi-Fast-Centers", ""))
            pi_confidence = float(headers.get("X-Vision-Pi-Fast-Confidence", "0"))
            if frame_seq <= last_seq:
                time.sleep(.02); continue
            last_seq = frame_seq
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            analysis = analyzer.analyze(frame)
            result = analysis.evidence
            red = planner.update(analyzer.red_band_layers, frame.shape[1], frame.shape[0])
            # Red bands own the calibrated I-course timing.  Geometry remains
            # a conservative warning when red tape is temporarily out of view.
            event = red.event
            if event == "CLEAR_ARM" and (result.junction_detected or result.endpoint_detected):
                event = "TURN_WINDOW_ARMED"
            annotated = render_complete_overlay(frame, analyzer, analysis.component_mask, result, red, event, pi_centers, pi_confidence)
            body = json.dumps({"token": args.token, "event": event, "frame_seq": frame_seq, "captured_at_ms": captured_at_ms, "event_at_ms": int(time.time() * 1000)}).encode()
            request = Request(args.pi_url.rstrip("/") + "/api/vision-adaptor/event", data=body, headers={"Content-Type": "application/json"}, method="POST")
            with urlopen(request, timeout=2) as response:
                accepted = response.read().decode()
            ok, preview = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 78])
            if ok:
                preview_request = Request(args.pi_url.rstrip("/") + "/api/vision-adaptor/preview", data=preview.tobytes(), headers={"Content-Type": "image/jpeg", "X-Vision-Adaptor-Token": args.token, "X-Vision-Frame-Seq": str(frame_seq), "X-Vision-Captured-At-Ms": str(captured_at_ms)}, method="POST")
                with urlopen(preview_request, timeout=2) as response:
                    preview_accepted = response.read().decode()
            else:
                preview_accepted = "jpeg_encode_failed"
            with args.log.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"time_ms": int(time.time() * 1000), "frame_seq": frame_seq, "event": event, "red_phase": red.phase, "red_layers": red.layer_count, "turn_y": red.turn_y, "turn_bottom_y": red.turn_bottom_y, "confidence": result.confidence, "pi_fast_confidence": pi_confidence, "accepted": accepted, "preview": preview_accepted}, ensure_ascii=False) + "\n")
            processed += 1
            if args.max_frames and processed >= args.max_frames:
                return
        except KeyboardInterrupt:
            return
        except Exception as exc:
            print(f"pc adaptor retry: {exc}", file=sys.stderr)
            time.sleep(.3)


if __name__ == "__main__":
    main()
