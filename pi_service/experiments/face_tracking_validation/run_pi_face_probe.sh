#!/usr/bin/env bash
set -Eeuo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${HERE}/face_probe_server.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --port 5058 "$@"
