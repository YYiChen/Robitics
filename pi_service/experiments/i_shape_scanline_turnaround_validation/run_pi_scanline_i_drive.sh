#!/usr/bin/env bash
# Isolated M1/M2 I-turn drive. It refuses a running main autonomous route.
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -u "${HERE}/scanline_i_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" --controller-url "http://127.0.0.1:5000" \
  --straight-pwm 120 --pivot-pwm 200 --pivot-min-seconds 2.5 \
  --enable-motors --headless --debug-web-port 5058 "$@"
