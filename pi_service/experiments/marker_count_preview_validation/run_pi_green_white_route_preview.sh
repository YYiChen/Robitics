#!/usr/bin/env bash
# Display-only green-floor / white-tape route preview. No motor commands.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_SERVICE_DIR="$(cd "${HERE}/../.." && pwd)"
WORKSPACE="$(cd "${PI_SERVICE_DIR}/.." && pwd)"

# The HSV config selects low-saturation bright tape only when a green floor is
# also visible. It does not use the old black-line Otsu threshold.
export TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.green_white_path.json"

exec "${HERE}/run_pi_marker_count_preview.sh"
