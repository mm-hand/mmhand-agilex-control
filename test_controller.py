from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from dataclasses import replace

import numpy as np

from config import ControlConfig
from control_loop import ArmController
from controller_state import ControlCommand, SnapshotStore
from pose_source import DemoPoseSource


class FakeArm:
    def __init__(self, side: str, config, motion, logger) -> None:
        self.side = side
        self.config = config
        self.firmware = "v120"
        self.q = np.zeros(7, dtype=float)
        self.moves = []
        self.holds = []
        self.sent = []
        self.closed = False

    def move_j_startup(self, side, q, stop_event=None) -> bool:
        if stop_event is not None and stop_event.is_set():
            return False
        self.moves.append(np.asarray(q, dtype=float).copy())
        self.q = np.asarray(q, dtype=float).copy()
        return True

    def prepare_teleop(self, side, stop_event=None):
        if stop_event is not None and stop_event.is_set():
            return None
        self.holds.append(self.q.copy())
        return self.q.copy()

    def current_joints(self, side):
        return self.q.copy()

    def refresh_feedback(self) -> bool:
        return True

    def update(self, left_q, right_q, frames, status, dt) -> None:
        self.sent.append((None if left_q is None else np.asarray(left_q).copy(), None if right_q is None else np.asarray(right_q).copy()))

    def close(self) -> None:
        self.closed = True


def _config() -> ControlConfig:
    config = ControlConfig()
    return replace(
        config,
        startup=replace(config.startup, enable_startup_pose=False),
        motion=replace(config.motion, command_hz=100.0),
    )


def _wait(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition not reached")


@contextmanager
def _controller():
    config = _config()
    store = SnapshotStore()
    pose = DemoPoseSource(config.pose_input)
    arms = {}

    def factory(side, arm_config, motion, logger):
        arms[side] = FakeArm(side, arm_config, motion, logger)
        return arms[side]

    controller = ArmController(config, logging.getLogger("test"), None, pose, store, arm_factory=factory)
    controller.start()
    try:
        yield controller, store, arms
    finally:
        controller.stop()
        controller.wait(timeout=3.0)


def test_pure_simulation_enters_teleop():
    with _controller() as (_controller_obj, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: not np.allclose(store.read().sim_q["left"], np.zeros(7)))
        assert not arms


def test_connect_left_then_confirm_routes_only_left():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "HANDOFF")
        assert store.read().handoff_sides == ("left",)
        assert not arms["left"].moves
        controller.submit(ControlCommand("CONFIRM", request_id=store.read().handoff_id))
        _wait(lambda: store.read().phase == "TELEOP")
        assert arms["left"].moves
        assert arms["left"].holds
        _wait(lambda: bool(arms["left"].sent))
        left_q, right_q = arms["left"].sent[-1]
        assert left_q is not None and right_q is None


def test_cancel_disconnects_without_motion():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "HANDOFF")
        controller.submit(ControlCommand("CANCEL", request_id=store.read().handoff_id))
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: arms["left"].closed)
        assert not arms["left"].moves


def test_both_arms_align_and_receive_commands():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "HANDOFF")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can1"))
        _wait(lambda: len(store.read().handoff_sides) == 2)
        controller.submit(ControlCommand("CONFIRM", request_id=store.read().handoff_id))
        _wait(lambda: store.read().phase == "TELEOP")
        assert arms["left"].moves and arms["right"].moves
        _wait(lambda: bool(arms["left"].sent) and bool(arms["right"].sent))
        assert arms["left"].sent[-1][0] is not None
        assert arms["right"].sent[-1][1] is not None


def test_duplicate_channel_is_rejected():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "HANDOFF")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can0"))
        _wait(lambda: store.read().arm_status["right"].state == "ERROR")
        assert "channel" in (store.read().arm_status["right"].error or "")


def test_stop_holds_simulation_and_following_arm():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "HANDOFF")
        controller.submit(ControlCommand("CONFIRM", request_id=store.read().handoff_id))
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: bool(arms["left"].sent))
        controller.submit(ControlCommand("STOP"))
        _wait(lambda: store.read().phase == "STOPPED")
        before = store.read().sim_q["left"]
        sent_before = len(arms["left"].sent)
        time.sleep(0.2)
        assert store.read().sim_q["left"] == before
        assert len(arms["left"].sent) > sent_before
