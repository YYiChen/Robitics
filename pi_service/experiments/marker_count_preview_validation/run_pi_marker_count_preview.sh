#!/usr/bin/env bash
# Display-only X-marker counter. It never creates a motor executor or sends card commands.
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_SERVICE_DIR="$(cd "${HERE}/../.." && pwd)"
WORKSPACE="$(cd "${PI_SERVICE_DIR}/.." && pwd)"
CONTINUOUS_DIR="${PI_SERVICE_DIR}/experiments/continuous_path_validation"
ROBOT_WEB_DIR="${PI_SERVICE_DIR}/robot_web"
WEB_PORT=5000
DEBUG_WEB_PORT=5055
CONTROLLER_URL="http://127.0.0.1:${WEB_PORT}"
TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.continuous_path.json"
LOG_DIR="${PI_SERVICE_DIR}/logs/marker_count_preview"
LOG_FILE="${LOG_DIR}/latest.log"
ROBOT_WEB_PID=""

cleanup() {
  # Display-only mode does not issue /api/drive, /api/stop, feed or deal calls.
  if [[ -n "${ROBOT_WEB_PID}" ]]; then
    kill "${ROBOT_WEB_PID}" >/dev/null 2>&1 || true
    wait "${ROBOT_WEB_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -f "${TRACK_CONFIG}" ]]; then
  echo "Missing marker-preview detector config: ${TRACK_CONFIG}" >&2
  exit 2
fi

if ! curl -fsS "${CONTROLLER_URL}/api/status" >/dev/null 2>&1; then
  echo "Starting robot_web only to provide the camera stream..."
  (
    cd "${ROBOT_WEB_DIR}"
    exec python3 -u app.py --port /dev/ttyACM0 --web-port "${WEB_PORT}"
  ) &
  ROBOT_WEB_PID=$!
  for _ in $(seq 1 40); do
    curl -fsS "${CONTROLLER_URL}/api/status" >/dev/null 2>&1 && break
    sleep .5
  done
fi

curl -fsS "${CONTROLLER_URL}/api/status" >/dev/null 2>&1 || { echo "Camera service did not start." >&2; exit 1; }
mkdir -p "${LOG_DIR}"
echo "Display-only marker preview: http://<Pi-IP>:${DEBUG_WEB_PORT}"
echo "Log: ${LOG_FILE}"

set +e
python3 -u "${CONTINUOUS_DIR}/continuous_path_runner.py" \
  --source "${CONTROLLER_URL}/video_feed" \
  --config "${TRACK_CONFIG}" \
  --process-fps 20 \
  --debug-web-port "${DEBUG_WEB_PORT}" \
  --headless \
  2>&1 | tee "${LOG_FILE}"
runner_status=${PIPESTATUS[0]}
set -e
python3 "${HERE}/summarize_marker_log.py" "${LOG_FILE}" || true
exit "${runner_status}"
