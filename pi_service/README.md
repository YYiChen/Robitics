# Robitics Pi Service

`pi_service/` is the complete Raspberry Pi software subproject for the Robitics
vehicle. Its governance, maintenance rules, tests, experiments, and runtime
boundaries are local to this directory.

Read [AGENTS.md](AGENTS.md) before changing this subproject.

## Current Formal Runtime

The normal control chain is:

```text
start_robot.sh
  -> robot_web/app.py
  -> active route tracker
  -> robot_web/controller.py
  -> USB serial
  -> Arduino Mega
```

The current default route mode is `end_line_turn_adaptor`. The service starts
vision paused; the operator uses the port-5000 console to arm or stop motor
control. Source support, passing tests, and successful HTTP/serial replies must
not be reported as successful physical motion without a real-car check.

Normal Pi startup:

```bash
cd /home/g11/Desktop/pi_service
chmod +x start_robot.sh
./start_robot.sh
```

This command can access the camera and Arduino. Do not run it as part of an
ordinary documentation, cleanup, or unit-test task.

## Directory Contract

| Path | Status | Responsibility |
| --- | --- | --- |
| `robot_web/` | Production | Flask API, browser UI, camera, route adapters, controller, telemetry, and safety-facing control flow |
| `tests/` | Production regression | Tests for formal service contracts |
| `experiments/` | Isolated validation | Algorithm, perception, calibration, hardware, and operator trials |
| `start_robot.sh` | Formal entry | Normal MJPEG control service; default route is `end_line_turn_adaptor` |
| `start_webrtc.sh` | Optional entry | H.264/WebRTC transport; mutually exclusive with the MJPEG camera owner |
| `verify_webrtc.sh` | Validation | WebRTC service verification after startup |
| `run_*_console.sh` | Explicit route selection | Operator launchers for a named route or validation mode |
| `install_dependencies.sh` | Setup | Pi OS packages and Python dependencies |
| `install_webrtc.sh` | Setup | Local MediaMTX installation under ignored `vendor/` |
| `robot_client.py` | Support library | HTTP client used by isolated tools and tests |
| `logs/`, `run/`, `runtime_logs/` | Runtime only | Logs and transient evidence; never source |

Files should not be moved between these areas only to make the tree look
cleaner. A move must have a functional target, updated imports and launchers,
and matching validation.

## Formal Modules

The main `robot_web/` responsibilities are intentionally separated:

| Module | Responsibility |
| --- | --- |
| `app.py` | CLI, Flask routes, service assembly, and route-mode selection |
| `controller.py` | Sole Pi-side Arduino serial controller and M1/M2 command path |
| `camera.py` | MJPEG CSI capture and camera settings |
| `dual_stream_camera.py` | Low-resolution video plus high-resolution JPEG capture |
| `webrtc_stream.py` | WebRTC transport integration |
| `end_line_turn_adaptor.py` | Current default white-line/end-line and operator-turn behavior |
| `scanline_i_route.py` | Scanline I-route adapter |
| `pc_vision_adaptor_route.py` | Legacy/optional PC vision event adapter |
| `autonomous_route.py` | Generic route preview, tuning, and run-gate support |
| `templates/`, `static/` | Port-5000 browser console |

Only `controller.py` should translate a route decision into the formal serial
motor path. Experiments must not silently become competing camera or motor
owners.

## Experiments

`experiments/` is grouped conceptually rather than by execution priority:

- drive and mapping: `car_drive_test`, `constant_drive_90_validation`,
  `differential_drive_mapping_validation`;
- route and line behavior: `continuous_path_validation`,
  `fixed_rectangle_validation`, `line_tracking_validation`,
  `straight_line_stop_validation`, and the `i_shape_*` families;
- terminal and marker perception: `end_line_turn_validation`,
  `marker_count_preview_validation`, `red_marker_feature_validation`;
- PC/Pi vision adapters: `pc_vision_adaptor_validation`,
  `fixed_course_offline_validation`;
- face tracking and motor sandboxes: `face_tracking_validation`,
  `pc_face_*`, `phone_face_*`;
- incomplete placeholders must remain visibly experimental until they gain an
  entry command, README, deterministic validation, and an explicit owner.

Each experiment owns its local files and README. Do not read or modify unrelated
experiments during a bounded target.

Promotion from an experiment to `robot_web/` requires:

1. a defined production responsibility;
2. deterministic focused tests;
3. integration tests under `tests/`;
4. preserved stop/timeout behavior;
5. documentation of Pi and physical-car validation;
6. removal or clear supersession of duplicate control ownership.

## Maintenance Workflow

For every target:

1. State the goal, owned files, read-only dependencies, validation, and hardware
   boundary in the task conversation.
2. Inspect `git status` and only the relevant production module or experiment.
3. Make one bounded class of change: feature, fix, tuning, cleanup, or move.
4. Run focused deterministic checks.
5. Review `git diff --check -- pi_service` and the owned diff.
6. Commit only the intended `pi_service` paths. Deployment and physical tests
   remain separate steps.

No repository-level `plan/` directory is required for this subproject. The
bounded target is recorded in the task conversation, the validation result in
the handoff, and the durable implementation state in Git.

## Validation Commands

From the repository root on Windows:

```powershell
py -3 -m unittest discover -s pi_service/tests -v
node --check pi_service/robot_web/static/app.js
git diff --check -- pi_service
git status --short --branch
```

Focused experiment:

```powershell
py -3 -m unittest discover -s pi_service/experiments/<target> -p "test_*.py" -v
```

On the Pi, validate a changed shell launcher before use:

```bash
bash -n pi_service/start_robot.sh
bash -n pi_service/<changed-launcher>.sh
```

Local tests may skip Flask, Picamera2, GPIO, I2C, serial, or MediaMTX behavior.
Report those as environment gaps. Never convert a skipped check into a claim of
Pi or vehicle validation.

## Configuration and Runtime Data

Deployment-specific state includes:

- `robot_web/drive_config.json`;
- legacy `robot_web/robot_config.json`;
- camera configuration;
- `logs/`, `run/`, and all experiment `runtime_logs/`;
- downloaded `vendor/` contents.

Preserve working Pi configuration during code synchronization. The committed
`drive_config.example.json` is a reference, not authorization to overwrite a
tuned vehicle.

Runtime logs are useful evidence but are not source code. Move selected evidence
into a deliberately named artifact or report only when a target explicitly
requires durable evidence.

## Git and Deployment Boundary

- Keep unrelated repository paths outside a `pi_service` change.
- Keep unfinished experiments off the deployable mainline until reviewed.
- Do not force-push shared history.
- Do not commit credentials or local authentication helpers.
- Synchronizing files to the Pi does not authorize service startup, firmware
  flashing, autonomous driving, or any motor action.
