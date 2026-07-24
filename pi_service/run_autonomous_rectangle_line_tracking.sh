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
for _ in $(seq 1 50); do
    if python3 - "${WEB_PORT}" <<'PY' >/dev/null 2>&1
import sys
from urllib.request import urlopen
urlopen(f"http://127.0.0.1:{sys.argv[1]}/api/status", timeout=0.2)
PY
    then
        ready=1
        break
    fi
    sleep 0.1
done

if [[ "${ready}" != "1" ]]; then
    echo "robot_web did not become ready. Check the serial port and whether port ${WEB_PORT} is already occupied." >&2
    exit 1
fi

echo "Autonomous tracking is active. Press Ctrl+C to STOP motors and exit."
cd "${PI_SERVICE_DIR}/experiments/line_tracking_validation"
python3 -u "${VISION_SCRIPT}" \
    --source "http://127.0.0.1:${WEB_PORT}/video_feed" \
    --config "${LINE_CONFIG}" \
    --process-fps 30 \
    --enable-motors \
    --controller-url "http://127.0.0.1:${WEB_PORT}" \
    --headless \
    "$@"
exit $?
