#!/usr/bin/env bash
# 连续路径循迹一键启动。所有参数在 tuning.py 中修改。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${HERE}/../../.." && pwd)"
TRACK_CONFIG="${WORKSPACE}/third_party/DeskMate-Advance/src/track_line/config.dark_line.json"

python3 -u "${HERE}/continuous_path_runner.py" \
  --source "http://127.0.0.1:5000/video_feed" \
  --controller-url "http://127.0.0.1:5000" \
  --config "${TRACK_CONFIG}" \
  --headless \
  --enable-motors
