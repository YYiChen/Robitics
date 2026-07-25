#!/usr/bin/env bash
# Explicit motor-enabled isolated I-shape turnaround validation.
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"
exec python3 -u "${HERE}/i_turnaround_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --controller-url "http://127.0.0.1:5000" \
  --config "${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.fixed_green_white_course.json" \
  --enable-motors --headless --debug-web-port 5057 "$@"
