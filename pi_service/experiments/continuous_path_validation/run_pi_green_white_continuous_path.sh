#!/usr/bin/env bash
# Green floor + white tape, with real motor control enabled.
# Ctrl+C always sends /api/stop through the underlying launcher.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"

# Do not change this to the old config.continuous_path.json: that file is for
# dark tape on a light floor. The green/white HSV thresholds live here.
export TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.fixed_green_white_course.json"

exec "${HERE}/run_pi_continuous_path.sh" "$@"
