# Scanline I-shape turnaround validation

This new isolated experiment does not modify `robot_web`, the main autonomous
route, or the third-party skeleton detector. It uses a near-anchored connected
component plus fixed scanlines: narrow vertical tape rows form the driving line;
a lower row whose tape width suddenly grows is an endpoint bar, never a left or
right branch to follow.

Preview only:

```bash
/home/g11/Desktop/pi_service/experiments/i_shape_scanline_turnaround_validation/run_pi_scanline_i_preview.sh
```

Open `http://<Pi-IP>:5058`. For the motor test, first stop the main route and
restart only the controller:

```bash
cd /home/g11/Desktop/pi_service
ROBOT_ENABLE_AUTONOMOUS_ROUTE=0 ./start_robot.sh
/home/g11/Desktop/pi_service/experiments/i_shape_scanline_turnaround_validation/run_pi_scanline_i_drive.sh
```

The drive script uses straight `120`, fixed right pivot `R=-200/L=+200`, and a
minimum pivot of `2.5 s`; it logs visual confidence, endpoint width/position,
PWM request/acknowledgement, Arduino output telemetry, and scene/line changes
to `pi_service/logs/i_shape_scanline_turnaround/latest.jsonl`.
