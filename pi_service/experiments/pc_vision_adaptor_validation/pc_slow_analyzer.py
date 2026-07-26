#!/usr/bin/env python3
"""Desktop companion: fetch Pi frames and post only validated visual events.

It is intentionally a first integration probe.  It runs the existing costly
green/white analyzer on the PC, records JSONL, and leaves wheel PWM entirely
to the Raspberry Pi adapter.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from queue import Empty, Full, Queue
import sys
from threading import Event, Lock, Thread
import time
from urllib.error import HTTPError
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


def _append_jsonl(path: Path, record: dict) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + "\n")


def _pi_runtime_snapshot(pi_url: str) -> dict:
    """Return only the Pi fields needed to correlate a PC decision to PWM."""
    with urlopen(pi_url.rstrip("/") + "/api/status", timeout=2) as response:
        payload = json.loads(response.read().decode("utf-8"))
    autonomous = payload.get("autonomous", {})
    robot = payload.get("robot", {})
    return {
        "frame": autonomous.get("frame"),
        "running": autonomous.get("running"),
        "enabled": autonomous.get("enabled"),
        "state": autonomous.get("state"),
        "motor": autonomous.get("motor"),
        "pc_event": autonomous.get("pc_event"),
        "pc_event_age_ms": autonomous.get("pc_event_age_ms"),
        "motor_output": robot.get("motor_output"),
        "arduino_reply": robot.get("reply"),
    }


def render_complete_overlay(frame, analyzer, component_mask, evidence, red, event: str, pi_centers, pi_confidence: float, debug: dict | None = None):
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
    debug = debug or {}
    cv2.rectangle(output, (8, 8), (1080, 202), (20, 20, 20), cv2.FILLED)
    cv2.putText(output, "PC FULL ANALYSIS -> PI SAFE MOTOR ADAPTOR", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, .63, (0, 220, 0), 2)
    cv2.putText(output, f"PC EVENT: {event}  RED: {red.phase} layers={red.layer_count} turn=({red.turn_y},{red.turn_bottom_y})", (16, 60), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 165, 255), 1)
    cv2.putText(output, f"PC: conf={evidence.confidence:.2f} line={evidence.line_center_x} bar={evidence.endpoint_detected}@{evidence.endpoint_y} junction={evidence.junction_detected}@{evidence.junction_y}", (16, 84), cv2.FONT_HERSHEY_SIMPLEX, .40, (255,255,255), 1)
    cv2.putText(output, f"PI FAST cyan-x: conf={pi_confidence:.2f} rows={len(pi_centers)} | PC green-dot: scanline", (16, 108), cv2.FONT_HERSHEY_SIMPLEX, .40, (255,255,0), 1)
    cv2.putText(output, f"LINK: pi_frame={debug.get('frame_seq')} capture_age={debug.get('capture_age_ms')}ms analysis={debug.get('analysis_ms')}ms event_rtt={debug.get('event_rtt_ms')}ms", (16, 132), cv2.FONT_HERSHEY_SIMPLEX, .38, (0,255,255), 1)
    pi_state = debug.get("pi_status", {})
    cv2.putText(output, f"PI ACK: {debug.get('event_response')} | state={pi_state.get('state')} motor={pi_state.get('motor')} output={pi_state.get('motor_output')}", (16, 156), cv2.FONT_HERSHEY_SIMPLEX, .38, (255,255,255), 1)
    cv2.putText(output, "yellow=tape / selected route; black=fit; red=green-field + red tape; orange=bar; purple=junction", (16, 180), cv2.FONT_HERSHEY_SIMPLEX, .38, (0,255,255), 1)
    return output


@dataclass(frozen=True)
class PreviewJob:
    frame_seq: int
    captured_at_ms: int
    frame: np.ndarray
    event: str
    red: object
    pi_centers: list[tuple[int, float, int]]
    pi_confidence: float
    control_ms: float


def _post_preview(pi_url: str, token: str, job: PreviewJob, annotated: np.ndarray) -> tuple[str, float | None]:
    ok, preview = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 78])
    if not ok:
        return "jpeg_encode_failed", None
    request = Request(
        pi_url.rstrip("/") + "/api/vision-adaptor/preview",
        data=preview.tobytes(),
        headers={"Content-Type": "image/jpeg", "X-Vision-Adaptor-Token": token, "X-Vision-Frame-Seq": str(job.frame_seq), "X-Vision-Captured-At-Ms": str(job.captured_at_ms)},
        method="POST",
    )
    started = time.monotonic()
    with urlopen(request, timeout=2) as response:
        return response.read().decode("utf-8"), round((time.monotonic() - started) * 1000, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description="PC slow visual analyser; never sends PWM")
    parser.add_argument("--pi-url", default="http://100.80.46.54:5000")
    parser.add_argument("--token", default="")
    parser.add_argument("--log", type=Path, default=HERE.parent / "runtime_logs" / "pc_slow_analyzer.jsonl")
    parser.add_argument("--status-every", type=int, default=10, help="fetch a compact Pi status after every N display frames; 0 disables polling")
    parser.add_argument("--max-frames", type=int, default=0, help="process this many frames then exit; 0 means run continuously")
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    control_analyzer, planner, last_seq, processed = GreenWhiteHybridScanlineAnalyzer(), TwoRedBandPlanner(), -1, 0
    preview_queue: Queue[PreviewJob] = Queue(maxsize=1)
    preview_stop, status_lock = Event(), Lock()
    latest_pi_status: dict = {}

    def publish_latest(job: PreviewJob) -> None:
        try:
            preview_queue.put_nowait(job)
        except Full:
            try:
                preview_queue.get_nowait()
            except Empty:
                pass
            preview_queue.put_nowait(job)

    def preview_worker() -> None:
        nonlocal latest_pi_status
        analyzer = GreenWhiteHybridScanlineAnalyzer()
        rendered = 0
        while not preview_stop.is_set():
            try:
                job = preview_queue.get(timeout=.1)
            except Empty:
                continue
            try:
                started = time.monotonic()
                analysis = analyzer.analyze(job.frame)
                evidence = analysis.evidence
                if args.status_every > 0 and rendered % args.status_every == 0:
                    status_started = time.monotonic()
                    status = _pi_runtime_snapshot(args.pi_url)
                    with status_lock:
                        latest_pi_status = status | {"status_rtt_ms": round((time.monotonic() - status_started) * 1000, 1)}
                with status_lock:
                    pi_status = dict(latest_pi_status)
                debug = {"frame_seq": job.frame_seq, "capture_age_ms": int(time.time() * 1000) - job.captured_at_ms, "analysis_ms": job.control_ms, "event_rtt_ms": None, "event_response": job.event, "pi_status": pi_status}
                annotated = render_complete_overlay(job.frame, analyzer, analysis.component_mask, evidence, job.red, job.event, job.pi_centers, job.pi_confidence, debug)
                response, preview_rtt_ms = _post_preview(args.pi_url, args.token, job, annotated)
                _append_jsonl(args.log, {"time_ms": int(time.time() * 1000), "kind": "pc_preview_cycle", "frame_seq": job.frame_seq, "event": job.event, "control_red_phase": job.red.phase, "control_ms": job.control_ms, "preview_analysis_ms": round((time.monotonic() - started) * 1000, 1), "preview_rtt_ms": preview_rtt_ms, "preview_response": response, "pi_status": pi_status})
                print(f"[PC PREVIEW] frame={job.frame_seq} analysis+render={round((time.monotonic() - started) * 1000, 1)}ms upload={preview_rtt_ms}ms")
                rendered += 1
            except Exception as exc:
                _append_jsonl(args.log, {"time_ms": int(time.time() * 1000), "kind": "pc_preview_error", "frame_seq": job.frame_seq, "error": repr(exc)})
                print(f"[PC PREVIEW] frame={job.frame_seq} error: {exc}", file=sys.stderr)

    worker = Thread(target=preview_worker, daemon=True, name="pc-vision-preview")
    worker.start()
    try:
      while True:
        try:
            stage = "frame_fetch"
            loop_started = time.monotonic()
            fetch_started = time.monotonic()
            with urlopen(args.pi_url.rstrip("/") + "/api/vision-adaptor/frame", timeout=2) as response:
                jpeg = response.read(); headers = response.headers
            fetch_ms = round((time.monotonic() - fetch_started) * 1000, 1)
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
            stage = "fast_red_control"
            control_started = time.monotonic()
            # This is the only visual path allowed to affect the motor.  It
            # intentionally excludes skeleton/junction/white-route topology.
            # The fast method still gates red pixels to the green course.
            layers = control_analyzer.detect_red_bands_fast(frame)
            red = planner.update(layers, frame.shape[1], frame.shape[0])
            event = red.event
            analysis_ms = round((time.monotonic() - control_started) * 1000, 1)
            body = json.dumps({"token": args.token, "event": event, "frame_seq": frame_seq, "captured_at_ms": captured_at_ms, "event_at_ms": int(time.time() * 1000)}).encode()
            request = Request(args.pi_url.rstrip("/") + "/api/vision-adaptor/event", data=body, headers={"Content-Type": "application/json"}, method="POST")
            stage = "event_post"
            event_started = time.monotonic()
            try:
                with urlopen(request, timeout=2) as response:
                    accepted = response.read().decode("utf-8")
            except HTTPError as exc:
                detail = exc.read().decode("utf-8", errors="replace").strip()
                _append_jsonl(args.log, {"time_ms": int(time.time() * 1000), "kind": "pc_to_pi_event_rejected", "frame_seq": frame_seq, "captured_at_ms": captured_at_ms, "capture_age_ms": int(time.time() * 1000) - captured_at_ms, "event": event, "analysis_ms": analysis_ms, "http_status": exc.code, "pi_response": detail})
                print(f"[PC→PI] frame={frame_seq} event={event} analysis={analysis_ms}ms HTTP {exc.code}: {detail or exc.reason}", file=sys.stderr)
                continue
            event_rtt_ms = round((time.monotonic() - event_started) * 1000, 1)
            stage = "preview_enqueue"
            publish_latest(PreviewJob(frame_seq, captured_at_ms, frame.copy(), event, red, pi_centers, pi_confidence, analysis_ms))
            with status_lock:
                pi_status = dict(latest_pi_status)
            record = {"time_ms": int(time.time() * 1000), "kind": "pc_control_cycle", "frame_seq": frame_seq, "captured_at_ms": captured_at_ms, "capture_age_ms": int(time.time() * 1000) - captured_at_ms, "total_cycle_ms": round((time.monotonic() - loop_started) * 1000, 1), "frame_fetch_ms": fetch_ms, "control_red_ms": analysis_ms, "event_rtt_ms": event_rtt_ms, "event": event, "red_phase": red.phase, "red_layers": red.layer_count, "turn_y": red.turn_y, "turn_bottom_y": red.turn_bottom_y, "red_layer_views": [{"y": layer.y, "bottom_y": layer.bottom_y, "span": layer.span, "fragments": layer.fragment_count} for layer in layers], "pi_fast_confidence": pi_confidence, "pi_fast_centers": pi_centers, "pi_event_response": accepted, "last_pi_status": pi_status}
            _append_jsonl(args.log, record)
            print(f"[PC CONTROL→PI] frame={frame_seq} fetch={fetch_ms}ms red={analysis_ms}ms event={event_rtt_ms}ms total={record['total_cycle_ms']}ms | event={event}")
            processed += 1
            if args.max_frames and processed >= args.max_frames:
                return
        except KeyboardInterrupt:
            return
        except HTTPError as exc:
            # Flask returns a useful JSON validation message on 400.  Preserve
            # it here; otherwise an expired frame and a malformed payload both
            # look like an unexplained "BAD REQUEST" on the PC.
            detail = exc.read().decode("utf-8", errors="replace").strip()
            print(f"pc adaptor retry: HTTP {exc.code}: {detail or exc.reason}", file=sys.stderr)
            time.sleep(.3)
        except Exception as exc:
            _append_jsonl(args.log, {"time_ms": int(time.time() * 1000), "kind": "pc_control_error", "stage": locals().get("stage", "unknown"), "frame_seq": locals().get("frame_seq"), "error_type": type(exc).__name__, "error": str(exc)})
            print(f"pc adaptor retry: {exc}", file=sys.stderr)
            time.sleep(.3)
    finally:
        preview_stop.set()
        worker.join(timeout=1.0)


if __name__ == "__main__":
    main()
