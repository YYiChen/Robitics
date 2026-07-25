#!/usr/bin/env bash
# Green floor + white tape using the current scanline I-turn state machine.
# Vision starts paused; press M on port 5000 to enable/stop M1/M2 driving.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${HERE}/start_robot.sh" \
  --enable-autonomous-route \
  --route-mode scanline_i_green_white \
  --route-process-fps 20 \
  "$@"
