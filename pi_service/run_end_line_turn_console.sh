#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_ROUTE_MODE=end_line_turn_adaptor
exec "${HERE}/start_robot.sh" "$@"
