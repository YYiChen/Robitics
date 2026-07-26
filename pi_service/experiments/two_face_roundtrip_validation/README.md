# Two-face roundtrip validation

Purpose: validate the pure route-memory sequence used by the formal end-line
tracker: outbound white-line following, face 1, the opposite-facing white-line
landmark, face 2, final white-line alignment, and an explicitly armed return.

Safety boundary: the tests import only the pure planner. They do not import the
camera, controller, Arduino, Flask service, or any motor-capable runner.

Entry command:

```powershell
py -3 -m unittest discover -s pi_service/experiments/two_face_roundtrip_validation -p "test_*.py" -v
```

Output: unittest transition assertions only.

Validation: both LEFT and RIGHT sweeps must mirror correctly; an incorrect
landmark type must be rejected; return following must remain separately armed.

Promotion criterion: integration in `robot_web/routes/end_line/tracker.py`
must preserve the M gate, heartbeat timeout for face turns, local timeout for
white-line turns, and explicit STOP behavior.
