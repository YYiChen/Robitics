#!/usr/bin/env bash
# M1/M2-only right-pivot probe. No camera, route detector, M3, or M4 is used.
set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Safety: raise the drive wheels first. Confirm M1/right is forward and M2/left is reverse. Ctrl+C sends STOP."
exec python3 -u "${HERE}/control_chain_probe.py" "$@"
