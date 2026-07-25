# I-shape turnaround validation

This folder is isolated from the main autonomous route program. Its default is
the current white-floor / black-tape camera setup (`config.dark_line.json`).

- `run_pi_i_turnaround_preview.sh`: vision only; browse `http://<Pi-IP>:5057`.
- `run_pi_control_chain_right_pivot_probe.sh`: first physical M1/M2 test. It sends only `R=+200, L=-200` for three seconds; it never opens a camera and never touches M3/M4.
- `run_pi_i_turnaround_drive.sh`: the I-route test, fixed at straight `120`, right pivot `R=+200/L=-200`, and a minimum pivot time of `2.5 s`.

Before either motor script, stop the 5000 process that was launched with
`--enable-autonomous-route`, then launch the isolated controller with
`cd /home/g11/Desktop/pi_service && ROBOT_ENABLE_AUTONOMOUS_ROUTE=0 ./start_robot.sh`.
Both motor scripts reject a status response that still has an in-process
autonomous route available: merely pausing it is not sufficient for this
isolated test.

Run the right-pivot probe with the wheels raised first, then in a clear low-speed
area.  Confirm physically that the right wheel moves forward, the left wheel
moves backward, and the chassis pivots right.  The JSONL records request/ack,
latest `/api/status` motor output, and Arduino reply; these telemetry values do
not replace physical confirmation.

The planner follows the long longitudinal stem, confirms a terminal transverse bar for several frames, pivots in one fixed direction for at least the calibrated 180-degree time, then accepts only a long non-transverse line to end the pivot. A pivot timeout stops the motors. Per-frame events are appended to `pi_service/logs/i_turnaround_validation/latest.jsonl`; each includes route state, transverse-bar position, requested/acknowledged PWM, latest Arduino output telemetry, and camera/route-change hints.
