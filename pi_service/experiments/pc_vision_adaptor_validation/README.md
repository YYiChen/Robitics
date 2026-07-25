# PC vision adaptor validation (default route mode)

This is the default autonomous route mode for the current green-floor course.
It moves the expensive HSV green-field, red-band, skeleton, and junction work
to the desktop PC.  The Raspberry Pi retains the only M1/M2 control path:

`camera -> Pi near-field scan -> Pi M-key gate -> controller.set_direct_drive -> Arduino`

The desktop may only send one of `SLOW_DOWN`, `TURN_WINDOW_ARMED`,
`BRAKE_NOW`, `PIVOT_REQUEST`, or `CLEAR_ARM` to `/api/vision-adaptor/event`.
It cannot submit PWM and it cannot bypass the M-key gate.  Events must carry a
fresh camera sequence/time and the shared token; old/out-of-order events are
rejected.  If an armed remote session becomes stale, the Pi stops rather than
driving blindly.

## Run

On the Pi, the normal `start_robot.sh` now starts `pc_vision_adaptor` by
default and still begins paused.  Open port 5000 and press **M** only after the
preview is healthy.  On the PC run:

```bash
cd pi_service/experiments/pc_vision_adaptor_validation
python3 pc_slow_analyzer.py --pi-url http://100.80.46.54:5000 --token "$ROBOT_PC_ADAPTOR_TOKEN"
```

Set the same `ROBOT_PC_ADAPTOR_TOKEN` in the Pi service environment and on the
PC.  Leave it empty only on an isolated lab network.  This script currently
proves transport and high-level arming; it does **not** claim physical turning
has been validated.

## Tests

```bash
python3 -m unittest test_protocol test_fast_line -v
```
