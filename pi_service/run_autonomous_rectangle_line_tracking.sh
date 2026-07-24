#!/usr/bin/env bash
# Run the complete rectangular line-following stack on the Raspberry Pi.
#
# The vision process talks only to 127.0.0.1:5000, so the vehicle does not
# depend on a PC, Wi-Fi latency, or the browser preview. Ctrl+C stops the
# motors through the vision process and then terminates the local web service.
set -Eeuo pipefail

PI_SERVICE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${PI_SERVICE_DIR}/.." && pwd)"
VISION_SCRIPT="${PI_SERVICE_DIR}/experiments/line_tracking_validation/live_rectangle_route_monitor.py"
LINE_CONFIG="${REPOSITORY_ROOT}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json"
WEB_PORT="${ROBOT_WEB_PORT:-5000}"
DEBUG_WEB_PORT="${LINE_TRACKING_DEBUG_WEB_PORT:-5051}"
SERIAL_PORT="${ROBOT_SERIAL_PORT:-/dev/ttyACM0}"
WEB_PID=""

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    if [[ -n "${WEB_PID}" ]] && kill -0 "${WEB_PID}" 2>/dev/null; then
        kill -TERM "${WEB_PID}" 2>/dev/null || true
        wait "${WEB_PID}" 2>/dev/null || true
    fi
    exit "${status}"
}
trap cleanup EXIT INT TERM

if [[ ! -f "${LINE_CONFIG}" ]]; then
    echo "Missing line-detector config: ${LINE_CONFIG}" >&2
    echo "Copy the repository's third_party/DeskMate-Advance/src/track_line directory too." >&2
    exit 2
fi

echo "Starting local robot service on port ${WEB_PORT} (serial: ${SERIAL_PORT})..."
(
    cd "${PI_SERVICE_DIR}/robot_web"
    exec python3 -u app.py --port "${SERIAL_PORT}" --web-port "${WEB_PORT}"
) &
WEB_PID=$!

ready=0
for _ in $(seq 1 120); do
    if python3 - "${WEB_PORT}" <<'PY' >/dev/null 2>&1
import json
import sys
from urllib.request import urlopen
with urlopen(f"http://127.0.0.1:{sys.argv[1]}/api/status", timeout=0.2) as response:
    status = json.load(response)
sys.exit(0 if status.get("robot", {}).get("arduino_online") else 1)
PY
    then
        ready=1
        break
    fi
    sleep 0.1
done

if [[ "${ready}" != "1" ]]; then
    echo "robot_web started, but Arduino did not become online within 12 seconds." >&2
    python3 - "${WEB_PORT}" <<'PY' >&2 || true
import json
import sys
from urllib.request import urlopen
try:
    with urlopen(f"http://127.0.0.1:{sys.argv[1]}/api/status", timeout=0.5) as response:
        robot = json.load(response).get("robot", {})
    print("serial=", robot.get("serial"), "arduino_online=", robot.get("arduino_online"))
    print("error=", robot.get("error"), "reply=", robot.get("reply"))
except Exception as exc:
    print("could not read robot status:", exc)
PY
    exit 1
fi

echo "Autonomous tracking is active. Open http://<Pi-IP>:${DEBUG_WEB_PORT} on a computer for the route-debug view."
echo "Press Ctrl+C to STOP motors and exit."
cd "${PI_SERVICE_DIR}/experiments/line_tracking_validation"
python3 -u "${VISION_SCRIPT}" \
    --source "http://127.0.0.1:${WEB_PORT}/video_feed" \
    --config "${LINE_CONFIG}" \
    --process-fps 30 \
    --enable-motors \
    --controller-url "http://127.0.0.1:${WEB_PORT}" \
    --debug-web-port "${DEBUG_WEB_PORT}" \
    --headless \
    "$@"
exit $?
