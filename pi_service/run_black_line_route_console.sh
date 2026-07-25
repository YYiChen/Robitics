#!/usr/bin/env bash
# White floor + black tape route preview inside the main port-5000 web service.
# This remains the explicit launcher; start_robot.sh now has the same black-line
# default. Vision starts paused. Press M on http://<Pi-IP>:5000 to drive/stop.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${HERE}/../third_party/DeskMate-Advance/src/track_line/config.dark_line.json"

exec "${HERE}/start_robot.sh" \
  --enable-autonomous-route \
  --route-config "${CONFIG}" \
  --route-process-fps 20 \
  "$@"
