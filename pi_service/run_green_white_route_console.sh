#!/usr/bin/env bash
# Green floor + white tape route preview inside the main port-5000 web service.
# Vision starts paused for safety. Open http://<Pi-IP>:5000 and press M to
# enable/stop autonomous driving without terminating this process.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${HERE}/../third_party/DeskMate-Advance/src/track_line/config.fixed_green_white_course.json"

exec "${HERE}/start_robot.sh" \
  --enable-autonomous-route \
  --route-config "${CONFIG}" \
  --route-process-fps 20 \
  "$@"
