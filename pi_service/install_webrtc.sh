#!/usr/bin/env bash
set -Eeuo pipefail

# Install MediaMTX beside this project. It is intentionally not committed:
# choose the newest release matching the Pi operating-system architecture.
root="$(cd "$(dirname "$0")" && pwd)"
case "$(uname -m)" in
  aarch64) asset_suffix="linux_arm64.tar.gz" ;;
  armv7l) asset_suffix="linux_armv7.tar.gz" ;;
  *) echo "Unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

command -v curl >/dev/null || { echo "curl is required" >&2; exit 1; }
command -v python3 >/dev/null || { echo "python3 is required" >&2; exit 1; }
sudo apt install -y ffmpeg

release_json="$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/releases/latest)"
asset_url="$(python3 -c '
import json, sys
suffix = sys.argv[1]
for asset in json.load(sys.stdin).get("assets", []):
    if asset.get("name", "").endswith(suffix):
        print(asset["browser_download_url"])
        break
' "$asset_suffix" <<<"$release_json")"
[[ -n "$asset_url" ]] || { echo "No MediaMTX asset found for $asset_suffix" >&2; exit 1; }

target="$root/vendor/mediamtx"
mkdir -p "$target"
archive="$(mktemp)"
trap 'rm -f "$archive"' EXIT
curl -fL "$asset_url" -o "$archive"
tar -xzf "$archive" -C "$target"
chmod +x "$target/mediamtx"
echo "Installed MediaMTX to $target/mediamtx"
