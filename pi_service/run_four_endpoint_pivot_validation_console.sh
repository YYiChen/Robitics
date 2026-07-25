#!/usr/bin/env bash
# Green floor + white tape: one-endpoint visual 90-degree pivot validation.
# The port-5000 preview starts paused; M controls M1/M2 only.  M3/M4 are not
# called by this validation mode.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${HERE}/start_robot.sh" \
  --enable-autonomous-route \
  --route-mode scanline_i_four_endpoint_green_white \
  --route-process-fps 20 \
  "$@"
