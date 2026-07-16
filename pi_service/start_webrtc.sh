#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "$0")" && pwd)"
mediamtx_bin="${MEDIAMTX_BIN:-$root/vendor/mediamtx/mediamtx}"
config="${MEDIAMTX_CONFIG:-$root/mediamtx.yml}"
camera_bin="${RPICAM_VID_BIN:-rpicam-vid}"
width="${ROBOT_WEBRTC_WIDTH:-1280}"
height="${ROBOT_WEBRTC_HEIGHT:-720}"
fps="${ROBOT_WEBRTC_FPS:-30}"
bitrate="${ROBOT_WEBRTC_BITRATE:-2500000}"
web_port="${ROBOT_WEB_PORT:-5000}"
serial_port="${ROBOT_SERIAL_PORT:-/dev/ttyACM0}"

[[ -x "$mediamtx_bin" ]] || { echo "MediaMTX is missing. Run ./install_webrtc.sh first." >&2; exit 1; }
[[ -f "$config" ]] || { echo "MediaMTX config not found: $config" >&2; exit 1; }
command -v "$camera_bin" >/dev/null || { echo "rpicam-vid is missing. Install rpicam-apps." >&2; exit 1; }

mkdir -p "$root/logs"
"$mediamtx_bin" "$config" >"$root/logs/mediamtx.log" 2>&1 &
mediamtx_pid=$!

# Pi 5 uses software H.264 encoding. --low-latency suppresses B-frames so the
# newest frame reaches WebRTC without an encoder reorder queue.
"$camera_bin" -t 0 -n --width "$width" --height "$height" --framerate "$fps" \
  --codec libav --low-latency --bitrate "$bitrate" --intra 15 \
  --libav-format mpegts -o 'udp://127.0.0.1:1234?pkt_size=1316' \
  >"$root/logs/rpicam-vid.log" 2>&1 &
camera_pid=$!

cleanup() {
  kill "$camera_pid" "$mediamtx_pid" 2>/dev/null || true
  wait "$camera_pid" "$mediamtx_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "$root/robot_web"
python3 -u app.py --port "$serial_port" --web-port "$web_port" \
  --video-backend webrtc --webrtc-width "$width" --webrtc-height "$height" \
  --webrtc-fps "$fps" --webrtc-bitrate "$bitrate" --webrtc-port 8889 --webrtc-path cam
