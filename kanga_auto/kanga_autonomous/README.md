# kanga_autonomous

Autonomous movement by publishing **Twist** commands to `/cmd_vel`. The existing **wheel_command_mapper** (kanga_drive) subscribes to `/cmd_vel` and sends velocity commands to the ODrive axes, so no changes to the drive stack are required.

## Editing the sequence (dev-friendly)

Edit **`kanga_autonomous/movement_sequence.py`** and change **`MOVEMENT_SEQUENCE`**. No recompile: edit and re-run the node.

**Helpers (easiest):**

| Helper     | Meaning              | Example        |
|-----------|----------------------|----------------|
| `FWD(t, v)`   | Forward for t sec, speed v (default 5) | `FWD(2)` or `FWD(2, 8)` |
| `BACK(t, v)`  | Backward             | `BACK(2)`      |
| `STOP(t)`     | Stop for t sec       | `STOP(1)`      |
| `ROT_L(t, v)` | Rotate left (CCW)    | `ROT_L(2)`     |
| `ROT_R(t, v)` | Rotate right (CW)    | `ROT_R(2)`     |
| `STRAFE_L(t,v)` / `STRAFE_R(t,v)` | Strafe left/right | `STRAFE_R(1, 4)` |

**Generic:** `cmd(t, x=0, y=0, z=0)` — duration `t`, then linear_x, linear_y, angular_z.

**Tuple shorthand:** `(duration, vx, vy, wz)` e.g. `(2, 5, 0, 0)` = 2 s forward at 5.

Example:

```python
MOVEMENT_SEQUENCE = [
    FWD(2), STOP(1), BACK(2), STOP(1),
    ROT_L(2), STOP(1), ROT_R(2), STOP(1),
]
```

Set **`LOOP_SEQUENCE = True`** to repeat the sequence indefinitely.

## Run

Requires the drive stack running (wheel_command_mapper + ODrive) so `/cmd_vel` is consumed.

```bash
# From workspace root (after colcon build and source)
ros2 launch kanga_autonomous autonomous.launch.py
```

Or run the node only:

```bash
ros2 run kanga_autonomous movement_sequence
```
