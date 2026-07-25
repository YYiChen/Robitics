#!/usr/bin/env bash
# 连续路径循迹一键启动。所有参数在 tuning.py 中修改。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"
TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json"
ROBOT_WEB_DIR="${WORKSPACE}/pi_service/robot_web"
WEB_PORT=5000
CONTROLLER_URL="http://127.0.0.1:5000"
ROBOT_WEB_PID=""
LOG_DIR="${WORKSPACE}/pi_service/logs/continuous_path"
LOG_FILE="${LOG_DIR}/latest.log"

cleanup() {
  # Always request a motor stop. Only terminate robot_web if this launcher
  # started it; an independently started service is left running.
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
  echo "Robot service is not running; starting it on port 5000..."
  (
    cd "${ROBOT_WEB_DIR}"
    exec python3 -u app.py --port /dev/ttyACM0 --web-port 5000
  ) &
  ROBOT_WEB_PID=$!
  for _attempt in $(seq 1 40); do
    if arduino_online >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done
  if ! arduino_online >/dev/null 2>&1; then
    echo "Arduino did not become online within 20 seconds; check /dev/ttyACM0 and robot_web log."
    exit 1
  fi
fi

mkdir -p "${LOG_DIR}"
echo "Writing continuous-path decisions to ${LOG_FILE}"
python3 -u "${HERE}/continuous_path_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --controller-url "${CONTROLLER_URL}" \
  --config "${TRACK_CONFIG}" \
  --headless \
  --enable-motors \
  2>&1 | tee "${LOG_FILE}"
