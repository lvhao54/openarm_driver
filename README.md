# OpenArm Driver

A Python library for controlling [OpenArm](https://github.com/enactic/openarm/), using [OpenArm CAN](https://github.com/enactic/openarm_can/).

## Quick start

TODO

## Install

```bash
pip install openarm-driver
```

## Sample usage

```python
import openarm_driver

arm = openarm_driver.SingleArmDriver("right_arm")
# You can also use your own config file as well.
# config = openarm_driver.Config("/path/to/config.yaml")
# arm = openarm_driver.SingleArmDriver("right_arm", config)

try:
    arm.start()
    while True:
        cur_position = arm.fetch_position()
        # Some process to calculate the next steps.
        next_positions = inference(cur_position)
        for next_postion in next_positions:
            arm.smooth_move(next_postion, hz=50, duration=1)
            # you can use simple command as well (Please be careful not to move the arm too much).
            # arm.send_position(next_postion)
finally:
    arm.stop()
```

## Dual-arm control

Use two ordinary `SingleArmDriver` instances in one application loop. The
[sample script](samples/dual_arm_motion.py) uses the existing driver without
subclassing or changing its implementation:

```bash
python samples/dual_arm_motion.py --config openarm_pedestal
```

This enables both arms, runs each arm's configured startup trajectory in sequence,
holds their measured positions for two seconds, then runs each configured stop
trajectory in sequence. The arms can move during startup and shutdown. Set
`--left-target` and/or `--right-target` to eight absolute positions in radians
(seven joints followed by the gripper) to move during the shared loop. An omitted
target holds that arm's position after startup. Use `--hz` and `--duration` to set
the command frequency and trajectory duration, and `--left-can-interface` /
`--right-can-interface` to override the configured CAN interfaces.

The core loop computes both targets from the same trajectory progress and calls
`left.send_position(...)`, then `right.send_position(...)`, checking each return
value. Each call includes its own safety checks, dispatch and feedback read.
Calling `left.smooth_move(...)` followed by `right.smooth_move(...)` would execute
the two trajectories one after the other instead.

If startup or a position command returns `False`, or an operation raises an
exception (including Ctrl+C), the script stops the loop and attempts to disable
both arms whose startup was attempted, without a return trajectory. Disable
failures are logged without skipping the other arm or hiding the original error.
Disabling removes holding torque. The script does not automatically resume.

The writes are sequential: a right-arm rejection or send failure can occur after
the left target was already sent. Independent safety clamping can produce
different progress on each arm; the script warns if the final dispatched command
differs from the requested endpoint. It does not verify physical arrival, provide
hardware synchronization, or check collisions between the arms.

For overlapping startup, motion, and shutdown, use the threaded demo:

```bash
python samples/dual_arm_parallel.py --config openarm_pedestal \
  --left-target 0 0 0 1.57 0 0 0 0 \
  --right-target 0 0 0 1.57 0 0 0 0
```

`dual_arm_parallel.py` gives each unchanged `SingleArmDriver` and CAN interface
its own worker thread. Their configured startup and shutdown trajectories run
concurrently. The target loop uses a shared clock and barriers at each step.
The same target, interface, frequency and duration options apply; omitted targets
hold the last position command dispatched by the startup trajectory. This keeps
the startup holding torque while motor feedback catches up. `--sync-timeout`
controls how long a worker waits for its peer at a barrier (default: 30 seconds).

A failure or Ctrl+C signals cancellation to both workers. An extra application
safety checker rejects subsequent commands, including commands inside the
original startup and shutdown trajectories; the existing safety checks remain
active. Each worker whose startup was attempted handles its own disable, and the
main thread waits for cleanup. Disable errors are logged, and the first failure
is preserved. Cancellation is cooperative: it cannot interrupt an in-flight or
blocked native CAN call, and disabling may fail if the interface is unavailable.

Calls still have ordinary operating-system and CAN scheduling skew; this is
concurrent software control, not hardware-synchronized or atomic dispatch.
Independent safety clamping and physical tracking errors can still give the arms
different progress. The two drivers must use separate CAN interfaces.

## Config

Please refer to the [default configuration](src/openarm_driver/configs/openarm_cell.yaml).

### Bundled configurations

The package bundles several configurations. Pass a bundled name to `Config()`
to select one, or pass a path to use your own file:

```python
import openarm_driver

openarm_driver.available_configs()
# ['openarm_cell', 'openarm_cell_higher_pd', 'openarm_pedestal']

config = openarm_driver.Config("openarm_pedestal")
arm = openarm_driver.SingleArmDriver("right_arm", config)

# Or make it the default for every driver created afterwards.
openarm_driver.set_default_config(config)
```

| Name | Description |
| --- | --- |
| `openarm_cell` | Default. OpenArm mounted on the cell frame. |
| `openarm_cell_higher_pd` | Same as `openarm_cell` with higher PD gains. |
| `openarm_pedestal` | OpenArm mounted on the pedestal (zero joint offsets). |

The default safety checks run in this order:

1. `JointPosChecker` clips commands to joint position limits.
2. `JointDeltaPosChecker` rejects excessive single-command jumps.
3. `JointVelocityChecker` limits the remaining command using the elapsed time.

`joint_velocity_limits` is specified in rad/s. `send_position()` measures the
elapsed command time automatically, so callers do not need to provide the node
control frequency. Custom configurations may omit this field to disable command
velocity limiting.

Elapsed command time is capped at 40 ms. This bounds the position increment
allowed by the velocity limiter after a scheduling pause or command gap.
At 250 Hz, a normal 4 ms interval still uses 4 ms in the calculation.

## Safety stops and recovery

`send_position()` returns `True` after dispatching the checked target, including
any safety clamping. A force-stop safety rejection returns `False` and latches
the reason in the read-only `arm.safety_stop_reason` property. The rejected target
is never dispatched. Further position commands return `False`, with warnings
at most once every two seconds while commands are attempted. This replaces the
previous `RuntimeError` for force-stop safety rejections.

The latch stops new position commands and leaves the last dispatched target in
place. It does not automatically disable motors or periodically resend that
target; actual holding behavior depends on the motor and communication state.
An explicit `stop()` skips the return trajectory after a latch, logs the reason,
and calls `disable_all()`. If a safety rejection interrupts a normal stop
trajectory, `stop()` still proceeds to disable the motors.

After inspecting and resolving the cause, recover with `stop()` followed by
`start()`. Starting clears the latch and synchronizes the command baseline to
the position read from the motors before sending the startup trajectory.
`safety_stop_reason` is `None` when no safety stop is latched. The existing
`get_health()` return structure remains `(motor_status, bus)`.

`smooth_move()`, `move_to_start_position()`, `move_to_stop_position()`, and
`start()` return `False` when their trajectory is rejected and `True` when it
finishes dispatching. Rejection stops the remaining trajectory steps. A failed
start leaves `started=False`, retains the reason, and blocks subsequent position
commands until recovery. Motors may still be enabled until `stop()` is called.
Successful dispatch does not verify physical arrival at the target.

Existing callers can continue to ignore these return values. Custom trajectory
hooks returning `None` remain supported; an explicit `False` or a latched safety
stop prevents successful startup. Configuration errors and CAN exceptions retain
their exception behavior. This change requires no additional node inputs or
metadata. It adds no fresh-feedback startup gate: cached feedback can still be
stale when motors are not responding. Health diagnostics remain observational.

## Development

### Test

```bash
uv sync
uv run pytest
```

### Release

```bash
git clone git@github.com:enactic/openarm_driver.git
cd openarm_driver
dev/release.sh ${VERSION} # e.g. dev/release.sh 1.0.0
```

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/can/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See [LICENSE.txt](LICENSE.txt) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
