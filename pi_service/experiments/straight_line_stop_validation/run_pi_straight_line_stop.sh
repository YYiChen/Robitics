#!/usr/bin/env bash
# Isolated minimal experiment: visible line -> straight correction; line end -> STOP.
set -Eeuo pipefail

EXPERIMENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_SERVICE_DIR="$(cd "${EXPERIMENT_DIR}/../.." && pwd)"
REPOSITORY_ROOT="$(cd "${PI_SERVICE_DIR}/.." && pwd)"
WEB_PORT="${ROBOT_WEB_PORT:-5000}"
DEBUG_WEB_PORT="${STRAIGHT_LINE_DEBUG_WEB_PORT:-5052}"
SERIAL_PORT="${ROBOT_SERIAL_PORT:-/dev/ttyACM0}"
CONFIG="${REPOSITORY_ROOT}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json"
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

[[ -f "${CONFIG}" ]] || { echo "Missing detector config: ${CONFIG}" >&2; exit 2; }
if pgrep -f 'live_rectangle_route_monitor.py' >/dev/null 2>&1; then
    echo "Old rectangle route monitor is still running. Stop it first; two vision programs must not command the same motors." >&2
    exit 3
fi
(
    cd "${PI_SERVICE_DIR}/robot_web"
    exec python3 -u app.py --port "${SERIAL_PORT}" --web-port "${WEB_PORT}"
) &
WEB_PID=$!

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
        break
    fi
    sleep 0.1
done

python3 - "${WEB_PORT}" <<'PY' >/dev/null 2>&1 || { echo "Arduino did not become online." >&2; exit 1; }
import json
import sys
from urllib.request import urlopen
with urlopen(f"http://127.0.0.1:{sys.argv[1]}/api/status", timeout=0.5) as response:
    status = json.load(response)
sys.exit(0 if status.get("robot", {}).get("arduino_online") else 1)
PY

echo "Straight-line stop validation active. Open http://<Pi-IP>:${DEBUG_WEB_PORT} for its live decision view."
echo "Ctrl+C sends STOP and exits."
cd "${EXPERIMENT_DIR}"
python3 -u straight_line_stop_runner.py \
    --source "http://127.0.0.1:${WEB_PORT}/video_feed" \
    --controller-url "http://127.0.0.1:${WEB_PORT}" \
    --config "${CONFIG}" \
    --process-fps 30 \
    --line-lost-stop-frames 5 \
    --straight-pwm 65 \
    --launch-pwm 155 \
    --minimum-correction-pwm 20 \
    --maximum-correction-pwm 60 \
    --debug-web-port "${DEBUG_WEB_PORT}" \
    --enable-motors \
    --headless \
    "$@"
