#!/usr/bin/env bash
# Isolated M1/M2 mapping check. It never starts camera, vision, route planning or card motors.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_SERVICE_DIR="$(cd "${HERE}/../.." && pwd)"
WEB_PORT=5000
CONTROLLER_URL="http://127.0.0.1:${WEB_PORT}"
LOG_DIR="${PI_SERVICE_DIR}/logs/differential_drive_mapping"
LOG_FILE="${LOG_DIR}/latest.log"
ROBOT_WEB_PID=""

cleanup() {
  curl -fsS -X POST "${CONTROLLER_URL}/api/stop" -H "Content-Type: application/json" -d '{}' >/dev/null 2>&1 || true
  if [[ -n "${ROBOT_WEB_PID}" ]]; then
    kill "${ROBOT_WEB_PID}" >/dev/null 2>&1 || true
    wait "${ROBOT_WEB_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

arduino_online() {
  python3 - "${WEB_PORT}" <<'PY'
import json
import sys
from urllib.request import urlopen

with urlopen(f"http://127.0.0.1:{sys.argv[1]}/api/status", timeout=.5) as response:
    status = json.load(response)
sys.exit(0 if status.get("robot", {}).get("arduino_online") else 1)
PY
}

if ! arduino_online >/dev/null 2>&1; then
  echo "Starting robot_web and waiting for Arduino..."
  (
    cd "${PI_SERVICE_DIR}/robot_web"
    exec python3 -u app.py --port /dev/ttyACM0 --web-port "${WEB_PORT}"
  ) &
  ROBOT_WEB_PID=$!
  for _ in $(seq 1 40); do
    arduino_online >/dev/null 2>&1 && break
    sleep .5
  done
fi

arduino_online >/dev/null 2>&1 || { echo "Arduino is not online; no drive command was sent." >&2; exit 1; }
mkdir -p "${LOG_DIR}"
echo "Safety: raise the drive wheels or keep clear space. The test lasts about 2.6 seconds. Ctrl+C stops M1/M2."
python3 -u "${HERE}/differential_drive_mapping_test.py" --controller-url "${CONTROLLER_URL}" "$@" 2>&1 | tee "${LOG_FILE}"
