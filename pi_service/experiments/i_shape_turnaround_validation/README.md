# I-shape turnaround validation

This folder is isolated from the main autonomous route program. Its default is
the current white-floor / black-tape camera setup (`config.dark_line.json`).

- `run_pi_i_turnaround_preview.sh`: vision only; browse `http://<Pi-IP>:5057`.
- `run_pi_i_turnaround_drive.sh`: explicit motor test. Do not run it together with the main autonomous tracker; first stop the main route service or leave its automatic drive paused.

The planner follows the long longitudinal stem, confirms a terminal transverse bar for several frames, pivots in one fixed direction for at least the calibrated 180-degree time, then accepts only a long non-transverse line to end the pivot. A pivot timeout stops the motors. Per-frame events are appended to `pi_service/logs/i_turnaround_validation/latest.jsonl`.
