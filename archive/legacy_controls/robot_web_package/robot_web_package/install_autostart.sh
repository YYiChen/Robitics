#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
PYTHON_BIN="$(command -v python3)"
SERVICE_FILE="/etc/systemd/system/robot-web.service"

sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=Raspberry Pi Robot Web Controller
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$PROJECT_DIR
ExecStart=$PYTHON_BIN -u $PROJECT_DIR/robot_control.py --port /dev/ttyACM0 --web-host 0.0.0.0 --web-port 5000
Restart=on-failure
RestartSec=3
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

sudo usermod -aG dialout,video "$RUN_USER"
sudo systemctl daemon-reload
sudo systemctl enable --now robot-web.service

echo "开机自启已安装并启动。"
echo "状态：sudo systemctl status robot-web.service"
echo "日志：journalctl -u robot-web.service -f"
echo "网页：http://100.80.46.54:5000/"
