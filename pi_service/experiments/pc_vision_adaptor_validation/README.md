# PC vision adaptor validation (default route mode)

This is the default autonomous route mode for the current green-floor course.
It moves the expensive HSV green-field, red-band, skeleton, and junction work
to the desktop PC.  The Raspberry Pi retains the only M1/M2 control path:

`camera -> Pi near-field scan -> Pi M-key gate -> controller.set_direct_drive -> Arduino`

The desktop may only send one of `SLOW_DOWN`, `TURN_WINDOW_ARMED`,
`BRAKE_NOW`, `PIVOT_REQUEST`, `REVERSE_REQUEST`, or `CLEAR_ARM` to
`/api/vision-adaptor/event`.
It cannot submit PWM and it cannot bypass the M-key gate.  Events must carry a
fresh camera sequence/time and the shared token; old/out-of-order events are
rejected.  If an armed remote session becomes stale, the Pi stops rather than
driving blindly.

### Fixed two-band course timing

The PC reconstructs red fragments into physical layers with a generous Y
cluster tolerance, so left/right fish-eye skew does not make one red band look
like two.  One early band means `SLOW_DOWN`.  When two layers have been seen
and the first leaves, the remaining turn band owns the final timing: at 50%
of image height it emits one `BRAKE_NOW` pulse, then Pi creeps; at 84% of image
height it emits one `PIVOT_REQUEST`; if it exits after reaching 70% but before
the pivot trigger, it emits one `REVERSE_REQUEST`.  Pi fixes the duration and
PWM of all three actions locally.

## Run

On the Pi, the normal `start_robot.sh` now starts `pc_vision_adaptor` by
default and still begins paused.  Open port 5000 and press **M** only after the
preview is healthy.  On the PC run:

```bash
cd pi_service/experiments/pc_vision_adaptor_validation
python3 pc_slow_analyzer.py --pi-url http://100.80.46.54:5000 --token "$ROBOT_PC_ADAPTOR_TOKEN"
```

Set the same `ROBOT_PC_ADAPTOR_TOKEN` in the Pi service environment and on the
PC.  Leave it empty only on an isolated lab network.  The PC now posts a
complete annotated JPEG to port 5000: PC masks/geometry/red-layer state and
cyan Pi fast-scan marks appear in the normal route-preview panel.  If PC
output is older than three seconds the panel falls back to the Pi-only overlay.
This does **not** claim physical turning has been validated.

## Tests

```bash
python3 -m unittest test_protocol test_fast_line test_red_band_planner -v
```
