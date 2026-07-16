#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "$0")" && pwd)"
mediamtx_bin="${MEDIAMTX_BIN:-$root/vendor/mediamtx/mediamtx}"
config="${MEDIAMTX_CONFIG:-$root/mediamtx.yml}"
width="${ROBOT_WEBRTC_WIDTH:-640}"
height="${ROBOT_WEBRTC_HEIGHT:-480}"
fps="${ROBOT_WEBRTC_FPS:-30}"
bitrate="${ROBOT_WEBRTC_BITRATE:-1500000}"
gop_frames="${ROBOT_WEBRTC_GOP_FRAMES:-8}"
highres_width="${ROBOT_HIGHRES_WIDTH:-1640}"
highres_height="${ROBOT_HIGHRES_HEIGHT:-1232}"
web_port="${ROBOT_WEB_PORT:-5000}"
serial_port="${ROBOT_SERIAL_PORT:-/dev/ttyACM0}"

[[ -x "$mediamtx_bin" ]] || { echo "MediaMTX is missing. Run ./install_webrtc.sh first." >&2; exit 1; }
[[ -f "$config" ]] || { echo "MediaMTX config not found: $config" >&2; exit 1; }
command -v ffmpeg >/dev/null || { echo "ffmpeg is missing. Run ./install_webrtc.sh first." >&2; exit 1; }

mkdir -p "$root/logs"
"$mediamtx_bin" "$config" >"$root/logs/mediamtx.log" 2>&1 &
mediamtx_pid=$!

cleanup() {
  kill "$mediamtx_pid" 2>/dev/null || true
  wait "$mediamtx_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "$root/robot_web"
python3 -u app.py --port "$serial_port" --web-port "$web_port" \
  --video-backend webrtc --webrtc-width "$width" --webrtc-height "$height" \
  --webrtc-fps "$fps" --webrtc-bitrate "$bitrate" --webrtc-gop-frames "$gop_frames" --webrtc-port 8889 --webrtc-path cam \
  --highres-width "$highres_width" --highres-height "$highres_height"
