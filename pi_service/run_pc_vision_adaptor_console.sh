#!/usr/bin/env bash
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_ROUTE_MODE=pc_vision_adaptor
exec "${HERE}/start_robot.sh" "$@"
