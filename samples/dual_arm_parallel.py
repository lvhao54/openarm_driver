# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run two unchanged SingleArmDriver instances in separate worker threads.

Startup and shutdown trajectories overlap. Target motion uses a shared clock
and a barrier each cycle. Each driver and CAN interface belongs to one worker.
This provides concurrent motion, not hardware-synchronized command dispatch.
"""

import argparse
import logging
import math
import threading
import time

import numpy as np

from openarm_driver import (
    Checker,
    CheckResult,
    CompositeChecker,
    Config,
    SingleArmDriver,
)

logger = logging.getLogger(__name__)


class CancellationChecker(Checker):
    """Reject subsequent position commands when the application cancels."""

    def __init__(self, cancelled):
        self.cancelled = cancelled

    def check(self, joint_positions, **kwargs):
        if self.cancelled.is_set():
            return CheckResult(
                is_safe=False,
                force_stop=True,
                message="Parallel demo cancelled",
            )
        return CheckResult(is_safe=True)


def finite_float(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("must be finite")
    return result


def positive_float(value):
    result = finite_float(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="openarm_pedestal", help="bundled configuration name or YAML path")
    parser.add_argument("--left-can-interface")
    parser.add_argument("--right-can-interface")
    parser.add_argument("--hz", type=positive_float, default=50.0)
    parser.add_argument("--duration", type=positive_float, default=2.0, help="seconds")
    parser.add_argument(
        "--sync-timeout",
        type=positive_float,
        default=30.0,
        help="maximum seconds to wait for the other worker at a barrier",
    )
    for side in ("left", "right"):
        parser.add_argument(
            f"--{side}-target",
            type=finite_float,
            nargs=8,
            metavar="RAD",
            help="absolute joint/gripper positions; default: hold after startup",
        )
    args = parser.parse_args(argv)
    if not math.isfinite(args.hz * args.duration):
        parser.error("hz * duration must be finite")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    config = Config(args.config)
    interfaces = (
        args.left_can_interface or config.get_can_interface("left_arm"),
        args.right_can_interface or config.get_can_interface("right_arm"),
    )
    if interfaces[0] == interfaces[1]:
        raise ValueError("Use separate CAN interfaces for the two arms")

    cancelled = threading.Event()
    errors = []
    error_lock = threading.Lock()
    epoch = [0.0]

    def set_epoch():
        epoch[0] = time.monotonic()

    # Reuse the phase barrier before enabling, before each target step and
    # before stopping. The separate motion barrier publishes one shared epoch.
    phase = threading.Barrier(2, timeout=args.sync_timeout)
    motion = threading.Barrier(2, action=set_epoch, timeout=args.sync_timeout)

    def cancel(exc):
        with error_lock:
            if not errors:
                errors.append(exc)
            cancelled.set()
        phase.abort()
        motion.abort()

    def check_cancelled():
        if cancelled.is_set():
            raise RuntimeError("Parallel demo cancelled")

    def run_arm(side, interface, requested, completed):
        arm = None
        startup_attempted = False
        stopped = False
        try:
            # Construct and access this driver's native CAN objects only here.
            arm = SingleArmDriver(side, config, can_interface=interface)
            arm.safety_checker = CompositeChecker(
                [CancellationChecker(cancelled), arm.safety_checker]
            )
            phase.wait()
            check_cancelled()
            startup_attempted = True
            logger.info("%s: starting", side)
            if arm.start() is False:
                raise RuntimeError(f"{side}: startup failed: {arm.safety_stop_reason}")
            check_cancelled()

            initial = np.asarray(arm.fetch_position(), dtype=float).copy()
            if initial.shape != (8,) or not np.all(np.isfinite(initial)):
                raise ValueError(
                    f"{side}: measured positions must be finite with shape (8,)"
                )
            target = initial.copy() if requested is None else np.array(requested)
            motion.wait()
            logger.info("%s: target motion", side)
            steps = max(1, math.ceil(args.hz * args.duration))
            for step in range(1, steps + 1):
                # Neither worker can run a step ahead of the other. Dispatch
                # within one step still has ordinary OS/CAN scheduling skew.
                phase.wait()
                progress = step / steps
                deadline = epoch[0] + args.duration * progress
                if cancelled.wait(max(0.0, deadline - time.monotonic())):
                    check_cancelled()
                check_cancelled()
                position = initial + (target - initial) * progress
                if arm.send_position(position) is False:
                    raise RuntimeError(f"{side}: {arm.safety_stop_reason}")

            if not np.allclose(arm.last_command, target, rtol=0, atol=1e-6):
                logger.warning(
                    "%s: final command differs from target after safety limiting; "
                    "the requested endpoint was not fully commanded",
                    side,
                )
            phase.wait()
            check_cancelled()
            logger.info("%s: stopping", side)
            arm.stop()
            if arm.safety_stop_reason is not None:
                raise RuntimeError(f"{side}: stop failed: {arm.safety_stop_reason}")
            stopped = True
        except threading.BrokenBarrierError:
            if not cancelled.is_set():
                cancel(TimeoutError(f"{side}: timed out waiting for the other arm"))
        except BaseException as exc:
            cancel(exc)
        finally:
            try:
                if startup_attempted and not stopped:
                    # No return trajectory after failure. A peer worker and the
                    # main thread never access this driver's CAN object directly.
                    try:
                        arm.openarm.disable_all()
                        arm.started = False
                    except BaseException:
                        logger.exception("%s: failed to disable motors", side)
            finally:
                completed.set()

    completions = [threading.Event(), threading.Event()]
    workers = [
        threading.Thread(
            target=run_arm,
            args=(side, interface, target, completed),
            name=f"parallel-{side}",
            daemon=False,
        )
        for side, interface, target, completed in zip(
            ("left_arm", "right_arm"),
            interfaces,
            (args.left_target, args.right_target),
            completions,
        )
    ]
    try:
        for worker in workers:
            worker.start()
    except BaseException as exc:
        cancel(exc)
    # A completion event is set only after that worker's cleanup. Keep signal
    # handling outside Thread.join() and preserve the first failure.
    for worker, completed in zip(workers, completions):
        if worker.ident is None:
            continue  # This worker was never started.
        while True:
            try:
                if completed.wait(timeout=0.05):
                    break
            except KeyboardInterrupt as exc:
                cancel(exc)
        worker.join(timeout=0)
    if errors:
        raise errors[0]
    return 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    raise SystemExit(main())
