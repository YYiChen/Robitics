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

The end-line route also supports checkpointed return:

1. press `M`, then `N` to start forward white-line following and recording;
2. every Arduino `DEAL:DONE` reply closes a route segment and creates a
   checkpoint;
3. press `R` to stop, perform a pulsed 180-degree turn, reacquire the white
   line, and traverse the recorded segments in reverse;
4. return motion remains vision-primary. Reversed wheel history contributes at
   most 50% (25% by default), and consecutive white-line loss stops the car.

Only M1/M2 samples, timing, line confidence, and integrated wheel-speed
estimates are recorded. M3/M4 commands are never replayed. Runtime evidence is
written to ignored `logs/end_line_return_route.json`. The file is diagnostic
evidence, not a localization guarantee; physical-car validation is required
before relying on return behavior.

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

The formal service is split into independently debuggable packages:

| Module | Responsibility |
| --- | --- |
| `app.py` | CLI, service assembly, lifecycle, and route-mode selection only |
| `api/` | Camera, control, route, and status HTTP endpoint registration |
| `controller.py` | Sole Pi-side Arduino serial and M1/M2 ownership facade |
| `control/` | Drive config, pure motor mapping, protocol parsing, servo, and card workflows |
| `camera.py` | MJPEG CSI hardware lifecycle |
| `dual_stream_camera.py` | WebRTC/H.264 plus high-resolution JPEG hardware lifecycle |
| `media/` | Camera profiles, settings persistence, colour correction, and transport metrics |
| `webrtc_stream.py` | WebRTC transport integration |
| `routes/common.py` | Shared motor run gate and latest-frame preview publisher |
| `routes/end_line/` | Current white-line/end-line perception, profiles, checkpoint recorder, and tracker |
| `routes/scanline/` | Scanline models, config, control, perception, planning, logging, and variants |
| `routes/pc_adaptor/` | PC event protocol plus Pi-local fast perception and motor tracker |
| `routes/generic/` | Generic continuous-path planner, control, marker count, and tracker |
| top-level route shims | Compatibility imports for existing scripts; no implementation ownership |
| `templates/`, `static/` | Port-5000 browser console |

Only `controller.py` should translate a route decision into the formal serial
motor path. Experiments must not silently become competing camera or motor
owners.

## Debug Map

Use the smallest module matching the symptom:

| Symptom or adjustment | Start with | Hardware-free check |
| --- | --- | --- |
| Wrong wheel sign/PWM | `control/motor_commands.py` | `test_control_modules.py` |
| Arduino reply parsing | `control/protocol.py` | `test_control_modules.py` |
| M3/M4 sequence or validation | `control/card_control.py` | `test_controller_persistence.py` |
| Servo command/heartbeat | `control/servo_control.py` | `test_controller_persistence.py` |
| Camera profile persistence | `media/settings.py` | `test_media_modules.py` |
| Camera bandwidth numbers | `media/metrics.py` | `test_media_modules.py` |
| Edge colour correction | `media/color_correction.py` | `test_camera_metrics.py` |
| HTTP request/response | matching file in `api/` | `test_api_structure.py` |
| Current terminal detection | `routes/end_line/perception.py` | end-line experiment tests |
| Current turn state/thread loop | `routes/end_line/tracker.py` | adaptor tuning tests |
| Checkpointed return recording/replay | `routes/end_line/return_route.py` | `test_return_route.py` |
| Scanline thresholds | `routes/scanline/config.py` | `test_scanline_modules.py` |
| Scanline wheel correction | `routes/scanline/control.py` | `test_scanline_modules.py` |
| Scanline evidence/state types | `routes/scanline/models.py` | scanline route tests |
| Scanline visual detection | `routes/scanline/perception.py` | scanline route tests |
| Scanline turn state machine | `routes/scanline/planner.py` | scanline route tests |
| Scanline audit records | `routes/scanline/logging.py` | scanline route tests |

Static path validation, which does not open hardware:

```bash
python3 pi_service/verify_paths.py
```

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

Formal runtime configuration now has one sectioned contract:

- `config/defaults.json` is the tracked shape and safe source default;
- `config/local.json` is the ignored Pi-specific override and the only file
  written by the formal web service;
- `config/local.example.json` shows the override structure without carrying
  deployed tuning;
- the owned sections are `drive`, `camera`, and `routes.end_line`.

On startup, the formal service imports existing values from
`robot_web/drive_config.json`, legacy `robot_web/robot_config.json`,
`robot_web/camera_config.json`, and the current end-line tuning/profile JSON
files only when the matching section is absent from `config/local.json`.
Existing unified sections always win. Keep those legacy files during the first
Pi rollout and compare the generated local override before retiring them.

Configuration precedence is:

1. an explicit CLI option such as `--drive-config`;
2. `config/local.json`;
3. `config/defaults.json`;
4. in-code validation defaults for missing or invalid fields.

Other deployment-specific state includes:

- `logs/`, `run/`, and all experiment `runtime_logs/`;
- downloaded `vendor/` contents.

Never put credentials in either configuration file. Preserve
`config/local.json` during code synchronization; copying tracked source files
must not replace a tuned vehicle.

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
