#!/usr/bin/env bash
# Display-only green-floor / white-tape route preview. No motor commands.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_SERVICE_DIR="$(cd "${HERE}/../.." && pwd)"
WORKSPACE="$(cd "${PI_SERVICE_DIR}/.." && pwd)"

# Fixed-course HSV tuning: it only accepts white tape inside the large green
# cloth and joins the small gaps caused by overlapped tape pieces.
export TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.fixed_green_white_course.json"

exec "${HERE}/run_pi_marker_count_preview.sh"
