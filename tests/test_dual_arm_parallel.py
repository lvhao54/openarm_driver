"""Exercise actual worker threads with simulated arm drivers, never real CAN."""

import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from openarm_driver import NullChecker

spec = importlib.util.spec_from_file_location(
    "dual_arm_parallel",
    Path(__file__).resolve().parents[1] / "samples/dual_arm_parallel.py",
)
parallel = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parallel)


@pytest.fixture
def rig(monkeypatch):
    state = SimpleNamespace(
        arms={},
        events=[],
        failures={},
        lock=threading.Lock(),
        overlap={
            name: threading.Barrier(2, timeout=1) for name in ("start", "send", "stop")
        },
        long_start=False,
    )

    class ConfigFake:
        def __init__(self, path):
            pass

        def get_can_interface(self, side):
            return {"left_arm": "can1", "right_arm": "can0"}[side]

    class ArmFake:
        def __init__(self, side, config, can_interface):
            self.arm_side = side
            self.can_interface = can_interface
            self.owner = threading.get_ident()
            self.started = False
            self.safety_stop_reason = None
            self.safety_checker = NullChecker()
            self.last_command = np.full(8, 0.2 if side == "left_arm" else -0.3)
            self.openarm = SimpleNamespace(disable_all=lambda: self.action("disable"))
            self.action("create")
            state.arms[side] = self

        def action(self, name):
            assert threading.get_ident() == self.owner
            with state.lock:
                state.events.append((self.arm_side, name, time.monotonic()))
            if name in state.overlap:
                # This fails if lifecycle or command calls are serialized.
                state.overlap[name].wait()
            failure = state.failures.get((self.arm_side, name))
            if isinstance(failure, BaseException):
                raise failure
            if failure is False:
                self.safety_stop_reason = "test safety stop"
                return False
            return True

        def check(self, target):
            result = self.safety_checker.check(target, driver=self, dt_s=0.02)
            if result.force_stop:
                self.safety_stop_reason = result.message
                return False
            return True

        def start(self):
            self.started = self.action("start")
            if state.long_start and self.arm_side == "right_arm":
                # Model original start()'s repeated checked trajectory commands.
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    if not self.check(self.last_command):
                        self.started = False
                        return False
                    time.sleep(0.001)
                pytest.fail("peer startup did not observe cancellation")
            return self.started

        def fetch_position(self):
            self.action("fetch")
            return self.last_command.copy()

        def send_position(self, target):
            if not self.action("send") or not self.check(target):
                return False
            self.last_command = target.copy()
            return True

        def stop(self):
            self.action("stop")
            self.check(self.last_command)
            self.started = False

    monkeypatch.setattr(parallel, "Config", ConfigFake)
    monkeypatch.setattr(parallel, "SingleArmDriver", ArmFake)
    return state


def args():
    return [
        "--duration",
        "0.04",
        "--hz",
        "50",
        "--sync-timeout",
        "1",
        "--left-target",
        *["0.3"] * 8,
        "--right-target",
        *["-0.5"] * 8,
    ]


def events(rig, action):
    return [side for side, name, _ in rig.events if name == action]


def test_all_phases_overlap_and_each_driver_has_one_owner(rig):
    assert parallel.main(args()) == 0
    left, right = rig.arms["left_arm"], rig.arms["right_arm"]
    assert left.owner != right.owner
    assert threading.get_ident() not in (left.owner, right.owner)
    assert (left.can_interface, right.can_interface) == ("can1", "can0")
    assert set(events(rig, "start")) == {"left_arm", "right_arm"}
    assert set(events(rig, "stop")) == {"left_arm", "right_arm"}
    assert events(rig, "send").count("left_arm") == 2
    assert events(rig, "send").count("right_arm") == 2
    np.testing.assert_allclose(left.last_command, 0.3)
    np.testing.assert_allclose(right.last_command, -0.5)
    assert not events(rig, "disable")


def test_default_targets_hold_after_parallel_startup(rig):
    assert parallel.main(["--duration", "0.005"]) == 0
    np.testing.assert_allclose(rig.arms["left_arm"].last_command, 0.2)
    np.testing.assert_allclose(rig.arms["right_arm"].last_command, -0.3)


@pytest.mark.parametrize("side", ["left_arm", "right_arm"])
@pytest.mark.parametrize("failure", [False, OSError("send failed")])
def test_send_failure_cancels_peer_and_both_workers_disable(rig, side, failure):
    rig.failures[(side, "send")] = failure
    with pytest.raises(RuntimeError if failure is False else OSError):
        parallel.main(args())
    assert sorted(events(rig, "disable")) == ["left_arm", "right_arm"]
    assert not events(rig, "stop")
    assert len(events(rig, "send")) == 2
    assert all(not arm.started for arm in rig.arms.values())


def test_start_failure_cancels_peer_inside_startup_trajectory(rig):
    rig.long_start = True
    original = OSError("left startup failed")
    rig.failures[("left_arm", "start")] = original
    with pytest.raises(OSError) as error:
        parallel.main(args())
    assert error.value is original
    assert rig.arms["right_arm"].safety_stop_reason == "Parallel demo cancelled"
    assert sorted(events(rig, "disable")) == ["left_arm", "right_arm"]
    assert not events(rig, "send")
    assert not events(rig, "stop")


def test_disable_failure_keeps_original_error_and_other_cleanup(rig, caplog):
    original = OSError("right send failed")
    rig.failures[("right_arm", "send")] = original
    rig.failures[("left_arm", "disable")] = OSError("left disable failed")
    with pytest.raises(OSError) as error:
        parallel.main(args())
    assert error.value is original
    assert sorted(events(rig, "disable")) == ["left_arm", "right_arm"]
    assert "left_arm: failed to disable motors" in caplog.text


@pytest.mark.parametrize("failure", [False, OSError("stop failed")])
def test_stop_failure_is_reported_and_failed_arm_is_disabled(rig, failure):
    rig.failures[("left_arm", "stop")] = failure
    with pytest.raises(RuntimeError if failure is False else OSError):
        parallel.main(args())
    assert "left_arm" in events(rig, "disable")
    assert set(events(rig, "stop")) == {"left_arm", "right_arm"}


def test_constructor_failure_releases_peer_without_enabling(rig):
    rig.failures[("right_arm", "create")] = OSError("missing right CAN")
    with pytest.raises(OSError, match="missing right CAN"):
        parallel.main(args())
    assert not events(rig, "start")
    assert not events(rig, "disable")


def test_main_thread_interrupt_cancels_workers(rig, monkeypatch):
    wait = threading.Event.wait
    interrupted = []
    main_thread = threading.get_ident()

    def interrupt_once(event, timeout=None):
        if threading.get_ident() == main_thread and timeout == 0.05 and not interrupted:
            interrupted.append(True)
            raise KeyboardInterrupt()
        return wait(event, timeout=timeout)

    monkeypatch.setattr(threading.Event, "wait", interrupt_once)
    with pytest.raises(KeyboardInterrupt):
        parallel.main(args())
    assert not any(
        thread.name.startswith("parallel-") for thread in threading.enumerate()
    )
    assert sorted(events(rig, "disable")) == sorted(events(rig, "start"))


def test_timeout_aborts_waiters_and_disables_started_arm(rig, monkeypatch):
    create = parallel.SingleArmDriver

    def with_slow_fetch(*positional, **keywords):
        arm = create(*positional, **keywords)
        original_fetch = arm.fetch_position
        if arm.arm_side == "right_arm":

            def fetch():
                time.sleep(0.05)
                return original_fetch()

            arm.fetch_position = fetch
        return arm

    monkeypatch.setattr(parallel, "SingleArmDriver", with_slow_fetch)
    with pytest.raises(TimeoutError, match="waiting for the other arm"):
        parallel.main(["--sync-timeout", "0.02", "--duration", "0.01"])
    assert sorted(events(rig, "disable")) == ["left_arm", "right_arm"]
    assert not events(rig, "send")


@pytest.mark.parametrize(
    "cli_args",
    [
        ["--hz", "0"],
        ["--duration", "nan"],
        ["--sync-timeout", "-1"],
        ["--left-target", *["inf"] * 8],
        ["--hz", "1e308", "--duration", "1e308"],
    ],
)
def test_invalid_input_never_opens_can(rig, cli_args):
    with pytest.raises(SystemExit):
        parallel.main(cli_args)
    assert rig.events == []


def test_same_can_interface_never_opens_can(rig):
    with pytest.raises(ValueError, match="separate CAN"):
        parallel.main(["--left-can-interface", "can0"])
    assert rig.events == []
