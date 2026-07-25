#!/usr/bin/env bash
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -u "${HERE}/scanline_i_runner.py" --source "http://127.0.0.1:5000/video_feed" --headless --debug-web-port 5058 "$@"
