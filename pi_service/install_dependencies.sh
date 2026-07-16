#!/usr/bin/env bash
set -Eeuo pipefail
sudo apt update
sudo apt install -y python3-flask python3-serial python3-opencv python3-picamera2 rpicam-apps python3-pip python3-pil python3-smbus i2c-tools
python3 -c 'import luma.oled' 2>/dev/null || python3 -m pip install --break-system-packages luma.oled
sudo usermod -aG dialout,video "$USER"
