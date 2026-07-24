#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
mkdir -p captures logs run

PYTHON_BIN="${PYTHON_BIN:-python3}"
SERIAL_PORT="${ROBOT_SERIAL_PORT:-/dev/ttyACM0}"
WEB_PORT="${ROBOT_WEB_PORT:-5000}"
PID_FILE="$PROJECT_DIR/run/robot.pid"
LOG_FILE="$PROJECT_DIR/logs/robot.log"

if [[ -f "$PID_FILE" ]]; then
  OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$OLD_PID" ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    echo "机器人网页服务已经在运行（PID $OLD_PID）。"
    echo "请直接打开：http://100.80.46.54:${WEB_PORT}/"
    exit 0
  fi
  rm -f "$PID_FILE"
fi

for module in flask serial; do
  if ! "$PYTHON_BIN" -c "import $module" >/dev/null 2>&1; then
    echo "缺少 Python 模块：$module"
    echo "请先运行：./install_dependencies.sh"
    exit 1
  fi
done

# 保持启动命令只有一个，同时把同一份输出写入终端和日志。
exec > >(tee -a "$LOG_FILE") 2>&1

echo $$ > "$PID_FILE"
echo "启动机器人网页控制器……"
echo "网页地址：http://100.80.46.54:${WEB_PORT}/"
echo "串口：$SERIAL_PORT"

# exec 后 Python 沿用当前 PID，stop_robot.sh 可以准确停止它。
exec "$PYTHON_BIN" -u "$PROJECT_DIR/robot_control.py" \
  --port "$SERIAL_PORT" \
  --web-host 0.0.0.0 \
  --web-port "$WEB_PORT"
