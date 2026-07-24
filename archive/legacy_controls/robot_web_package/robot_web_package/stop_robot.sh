#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$PROJECT_DIR/run/robot.pid"

if systemctl is-active --quiet robot-web.service 2>/dev/null; then
  sudo systemctl stop robot-web.service
  echo "robot-web.service 已停止。"
  exit 0
fi

if [[ -f "$PID_FILE" ]]; then
  PID="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    kill -INT "$PID"
    echo "已请求安全停车并停止网页服务。"
    exit 0
  fi
fi

echo "没有发现正在运行的机器人网页服务。"
