#!/usr/bin/env bash
# 连续路径循迹一键启动。所有参数在 tuning.py 中修改。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"
TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json"
ROBOT_WEB_DIR="${WORKSPACE}/pi_service/robot_web"
CONTROLLER_URL="http://127.0.0.1:5000"
ROBOT_WEB_PID=""

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

if ! curl -fsS "${CONTROLLER_URL}/api/status" >/dev/null; then
  echo "Robot service is not running; starting it on port 5000..."
  (
    cd "${ROBOT_WEB_DIR}"
    exec python3 -u app.py --port /dev/ttyACM0 --web-port 5000
  ) &
  ROBOT_WEB_PID=$!
  for _attempt in $(seq 1 20); do
    if curl -fsS "${CONTROLLER_URL}/api/status" >/dev/null; then
      break
    fi
    sleep 0.5
  done
  if ! curl -fsS "${CONTROLLER_URL}/api/status" >/dev/null; then
    echo "Robot service did not become ready; inspect the log above."
    exit 1
  fi
fi

python3 -u "${HERE}/continuous_path_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --controller-url "${CONTROLLER_URL}" \
  --config "${TRACK_CONFIG}" \
  --headless \
  --enable-motors
