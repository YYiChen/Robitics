#!/usr/bin/env bash
set -Eeuo pipefail
sudo apt update
sudo apt install -y python3-flask python3-serial python3-opencv python3-picamera2
sudo usermod -aG dialout,video "$USER"
echo
printf '%s\n' "依赖安装完成。若刚加入 dialout/video 用户组，请注销并重新登录，或重启树莓派。"
