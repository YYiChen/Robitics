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


def main() -> None:
    parser = argparse.ArgumentParser(description="PC slow visual analyser; never sends PWM")
    parser.add_argument("--pi-url", default="http://100.80.46.54:5000")
    parser.add_argument("--token", default="")
    parser.add_argument("--log", type=Path, default=Path("runtime_logs/pc_slow_analyzer.jsonl"))
    args = parser.parse_args()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    analyzer, last_seq = GreenWhiteHybridScanlineAnalyzer(), -1
    while True:
        try:
            with urlopen(args.pi_url.rstrip("/") + "/api/vision-adaptor/frame", timeout=2) as response:
                jpeg = response.read(); headers = response.headers
            frame_seq = int(headers.get("X-Vision-Frame-Seq", "-1"))
            captured_at_ms = int(headers.get("X-Vision-Captured-At-Ms", "0"))
            if frame_seq <= last_seq:
                time.sleep(.02); continue
            last_seq = frame_seq
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            result = analyzer.analyze(frame).evidence
            event = "TURN_WINDOW_ARMED" if result.junction_detected or result.endpoint_detected else "CLEAR_ARM"
            body = json.dumps({"token": args.token, "event": event, "frame_seq": frame_seq, "captured_at_ms": captured_at_ms, "event_at_ms": int(time.time() * 1000)}).encode()
            request = Request(args.pi_url.rstrip("/") + "/api/vision-adaptor/event", data=body, headers={"Content-Type": "application/json"}, method="POST")
            with urlopen(request, timeout=2) as response:
                accepted = response.read().decode()
            with args.log.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"time_ms": int(time.time() * 1000), "frame_seq": frame_seq, "event": event, "confidence": result.confidence, "accepted": accepted}, ensure_ascii=False) + "\n")
        except KeyboardInterrupt:
            return
        except Exception as exc:
            print(f"pc adaptor retry: {exc}", file=sys.stderr)
            time.sleep(.3)


if __name__ == "__main__":
    main()
