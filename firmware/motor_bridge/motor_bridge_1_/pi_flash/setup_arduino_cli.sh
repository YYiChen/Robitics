#!/usr/bin/env bash
# Install the Raspberry Pi upload prerequisites for Arduino Mega firmware.
set -Eeuo pipefail

CLI_DIR="$HOME/.local/bin"
CLI="$CLI_DIR/arduino-cli"

if [[ ! -x "$CLI" ]]; then
  echo "Installing Arduino CLI into $CLI_DIR ..."
  mkdir -p "$CLI_DIR"
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | BINDIR="$CLI_DIR" sh
fi

"$CLI" core update-index
"$CLI" core install arduino:avr
"$CLI" lib install "Adafruit Motor Shield library"
"$CLI" lib install Servo

echo
echo "Arduino CLI is ready: $CLI"
echo "The upload account must have serial-port permission."
if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  echo "Serial permission: OK ($USER is in dialout)."
else
  echo "Run this once, then log out and log in again (or reboot):"
  echo "  sudo usermod -a -G dialout $USER"
fi
