#!/usr/bin/env bash
# Isolated camera-only I-shape turnaround preview. It never drives motors.
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"
exec python3 -u "${HERE}/i_turnaround_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --config "${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json" \
  --headless --debug-web-port 5057 "$@"
