#!/usr/bin/env bash
set -Eeuo pipefail
sudo apt update
sudo apt install -y python3-flask python3-serial python3-opencv python3-picamera2 rpicam-apps
sudo usermod -aG dialout,video "$USER"
