# Differential drive mapping validation

This isolated test verifies whether the Arduino channels labelled `M1=right` and `M2=left` actually drive the physical right and left wheel groups. It does not open the camera, run vision, plan a route, or operate the card mechanism.

Run on the Pi:

```bash
/home/g11/Desktop/pi_service/experiments/differential_drive_mapping_validation/run_pi_differential_drive_mapping_test.sh
```

The sequence is deliberately short:

1. `right=0`, `left=180` for 5 seconds;
2. stop for one second;
3. `right=180`, `left=0` for 5 seconds;
4. final stop.

Lift the drive wheels or leave clear space. `Ctrl+C` sends `/api/stop`. The result is logged to `/home/g11/Desktop/pi_service/logs/differential_drive_mapping/latest.log`.
