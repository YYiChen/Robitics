#!/usr/bin/env bash
set -Eeuo pipefail
cd "$(dirname "$0")"
exec python3 -u app.py --port "${ROBOT_SERIAL_PORT:-auto}" --web-port "${ROBOT_WEB_PORT:-5050}" "$@"
