#!/usr/bin/env bash
# Explicit motor-enabled isolated I-shape turnaround validation.
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"
# Fixed starting values for the real vehicle: enough torque to start from
# rest and a strong signed differential pair for an in-place 180-degree turn.
exec python3 -u "${HERE}/i_turnaround_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --controller-url "http://127.0.0.1:5000" \
  --config "${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json" \
  --straight-pwm 120 \
  --pivot-pwm 200 \
  --pivot-min-seconds 2.5 \
  --enable-motors --headless --debug-web-port 5057 "$@"
