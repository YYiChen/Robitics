#!/usr/bin/env bash
set -Eeuo pipefail

web_port="${ROBOT_WEB_PORT:-5000}"
webrtc_port="${ROBOT_WEBRTC_PORT:-8889}"
stream_path="${ROBOT_WEBRTC_PATH:-cam}"
status_url="http://127.0.0.1:${web_port}/api/status"
webrtc_url="http://127.0.0.1:${webrtc_port}/${stream_path}/"

command -v curl >/dev/null || { echo "curl is required" >&2; exit 1; }
command -v python3 >/dev/null || { echo "python3 is required" >&2; exit 1; }

status_json="$(curl --fail --silent --show-error "$status_url")"
python3 -c '
import json, sys
data = json.load(sys.stdin)
camera = data.get("camera", {})
if camera.get("transport") != "webrtc":
    raise SystemExit("Flask is not in WebRTC mode: " + str(camera.get("transport")))
print("Flask backend:", camera.get("transport"))
print("Camera status:", camera.get("status"))
print("Camera online:", camera.get("online"))
print("H.264 target:", camera.get("resolution"), "@", camera.get("sensor_target_fps"), "FPS")
if not camera.get("highres_available"):
    raise SystemExit("High-resolution JPEG channel is not enabled")
print("High JPEG target:", camera.get("highres_profile", {}).get("resolution"), "@", camera.get("highres", {}).get("target_fps"), "FPS")
' <<<"$status_json"

curl --fail --silent --show-error --output /dev/null "$webrtc_url"
curl --fail --silent --show-error --output /dev/null "http://127.0.0.1:${web_port}/api/camera/highres/latest"
host_ip="$(hostname -I | awk '{print $1}')"
echo "MediaMTX WebRTC page: $webrtc_url"
echo "DL RTSP stream: rtsp://${host_ip}:8554/${stream_path}"
echo "PASS: WebRTC service endpoints are reachable locally."
