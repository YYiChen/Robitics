#!/usr/bin/env bash
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROUTE_ENABLED="${ROBOT_ENABLE_AUTONOMOUS_ROUTE:-1}"
ROUTE_CONFIG="${ROBOT_ROUTE_CONFIG:-${HERE}/../third_party/DeskMate-Advance/src/track_line/config.dark_line.json}"
ROUTE_FPS="${ROBOT_ROUTE_PROCESS_FPS:-20}"

# The main console owns the camera, Arduino, live route preview, and M-key
# motor gate.  Vision always starts paused; pressing M in the port-5000 page
# alone enables or stops automatic M1/M2 control.
route_args=()
if [[ "${ROUTE_ENABLED}" == "1" ]]; then
  route_args=(--enable-autonomous-route --route-config "${ROUTE_CONFIG}" --route-process-fps "${ROUTE_FPS}")
elif [[ "${ROUTE_ENABLED}" != "0" ]]; then
  echo "ROBOT_ENABLE_AUTONOMOUS_ROUTE must be 0 or 1" >&2
  exit 2
fi

cd "${HERE}/robot_web"
exec python3 -u app.py --port "${ROBOT_SERIAL_PORT:-/dev/ttyACM0}" --web-port "${ROBOT_WEB_PORT:-5000}" "${route_args[@]}" "$@"
