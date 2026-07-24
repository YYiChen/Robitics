#!/usr/bin/env bash
# Compile and upload the adjacent motor_bridge_1_ sketch to an Arduino Mega.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FQBN="${ARDUINO_FQBN:-arduino:avr:mega}"
PORT="${1:-${ARDUINO_PORT:-}}"

if [[ ! -x "$CLI" ]]; then
  echo "Arduino CLI is not installed. Run: ./pi_flash/setup_arduino_cli.sh" >&2
  exit 1
fi

if [[ -z "$PORT" ]]; then
  candidates=(/dev/ttyACM* /dev/ttyUSB*)
  found=()
  for candidate in "${candidates[@]}"; do
    [[ -e "$candidate" ]] && found+=("$candidate")
  done
  if [[ ${#found[@]} -ne 1 ]]; then
    echo "Unable to select one Arduino port. Available ports: ${found[*]:-(none)}" >&2
    echo "Use: ./pi_flash/flash_mega.sh /dev/ttyACM0" >&2
    exit 1
  fi
  PORT="${found[0]}"
fi

if [[ ! -e "$PORT" ]]; then
  echo "Serial port does not exist: $PORT" >&2
  exit 1
fi

if command -v fuser >/dev/null 2>&1 && fuser -s "$PORT"; then
  echo "Serial port $PORT is in use. Stop the robot web service first (Ctrl+C), then retry." >&2
  exit 1
fi

echo "Compiling motor bridge for $FQBN ..."
"$CLI" compile --fqbn "$FQBN" "$SKETCH_DIR"

echo "Uploading to $PORT ..."
"$CLI" upload --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo "Upload complete. Restart the robot web service after the Arduino reconnects."
