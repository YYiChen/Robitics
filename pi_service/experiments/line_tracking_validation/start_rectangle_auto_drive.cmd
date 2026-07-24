@echo off
setlocal
cd /d "%~dp0"
py -3 -u live_rectangle_route_monitor.py --source "http://100.80.46.54:5000/video_feed" --config "..\..\..\third_party\DeskMate-Advance\src\track_line\config.dark_line.json" --process-fps 10 --enable-motors --controller-url "http://100.80.46.54:5000"
set "exitCode=%ERRORLEVEL%"
if not "%exitCode%"=="0" pause
exit /b %exitCode%
