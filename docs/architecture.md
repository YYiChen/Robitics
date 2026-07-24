# Robitics architecture

## Deployment boundary

The deployed robot is deliberately small:

```text
Browser -> pi_service/robot_web -> USB serial -> firmware/motor_bridge -> hardware
                 |                                  |
                 +-> CSI camera / MJPEG or WebRTC    +-> drive, card motors, PID, sensors
```

`pi_service/start_robot.sh` is the normal MJPEG entry point. `start_webrtc.sh` is an alternative video transport, not a second service to run at the same time. The browser owns manual key state; `controller.py` owns serial translation and watchdog heartbeats; the Arduino owns raw motor output, encoder PID, servo smoothing and forward-obstacle blocking.

## Canonical directories

| Path | Role | Deployment status |
| --- | --- | --- |
| `firmware/motor_bridge/` | Arduino Mega production firmware and flash helper | Formal firmware |
| `pi_service/robot_web/` | Flask UI, serial controller, CSI camera, OLED and WebRTC adapter | Formal Pi service |
| `pi_service/tests/` | Service regression tests | Formal verification |
| `docs/` | Serial protocol, architecture and team material | Documentation |
| `tools/windows_recorder/` | Optional Windows-side MJPEG recorder | Optional workstation tool |
| `firmware/experiments/` | Isolated motor/card hardware sketches | Never flash as the default firmware |
| `pi_service/experiments/` | Isolated car-drive and line-tracking validations | Not connected to automatic motion |
| `third_party/` | Independently versioned external source references | Not deployed directly |
| `hardware/cad/` | CAD, STL and assembly assets | Mechanical reference |
| `archive/` | Course material, legacy controls, snapshots, release artifacts and capture evidence | Historical reference |

## Motor ownership

The current production firmware assigns M1 to right drive, M2 to left drive, M3 to timed card feeding and M4 to timed card dealing. The formal firmware is the only place where that contract should change. Older sketches may use different assignments and are therefore under `firmware/experiments/` or `archive/`.

## Line-tracking boundary

`pi_service/experiments/line_tracking_validation/` is a pure-Python state-machine experiment. It has no camera, GPIO, serial or motor I/O. The external OpenCV detector under `third_party/DeskMate-Advance/src/track_line/` is perception-only. A future live line-following feature must add a vision adapter, route/state planner and an explicit auto/manual safety arbiter before it can send actions through `controller.py`.

## Git boundary

The root repository owns the Robitics source, documentation, mechanical assets and archive. `deskmate/` and `third_party/DeskMate-Advance/` are independent repositories with their own histories; do not add them as embedded Git directories or deploy them as part of the Pi service.
