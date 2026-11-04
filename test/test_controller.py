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
        self.transport_dead = None

    def transport_error(self):
        return self.transport_dead

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

    controller = ArmController(config, logging.getLogger("test"), pose, store, arm_factory=factory)
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


def test_connect_latches_stop_without_motion():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        assert store.read().arm_status["left"].state == "STOPPED"
        assert not arms["left"].moves


def test_connect_second_arm_while_stopped():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can1"))
        _wait(lambda: store.read().arm_status["right"].state == "STOPPED")
        assert store.read().phase == "STOPPED"
        assert set(arms) == {"left", "right"}


def test_start_returns_simulation_and_arm_to_startup():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        assert len(arms["left"].moves) == 1
        assert np.allclose(arms["left"].moves[0], np.asarray(ControlConfig().startup.left_pose_rad))
        assert store.read().arm_status["left"].state == "TELEOP"


def test_start_moves_both_arms():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can1"))
        _wait(lambda: store.read().arm_status["right"].state == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        assert len(arms["left"].moves) == 1
        assert len(arms["right"].moves) == 1
        assert store.read().arm_status["left"].state == "TELEOP"
        assert store.read().arm_status["right"].state == "TELEOP"
        _wait(lambda: bool(arms["left"].sent) and bool(arms["right"].sent))
        left_q, right_q = arms["left"].sent[-1]
        assert left_q is not None and right_q is None
        left_q, right_q = arms["right"].sent[-1]
        assert left_q is None and right_q is not None


def test_duplicate_channel_is_rejected():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can0"))
        _wait(lambda: store.read().arm_status["right"].state == "ERROR")
        assert "channel" in (store.read().arm_status["right"].error or "")


def test_stop_holds_simulation_and_following_arm():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: bool(arms["left"].sent))
        controller.submit(ControlCommand("STOP"))
        _wait(lambda: store.read().phase == "STOPPED")
        before = store.read().sim_q["left"]
        sent_before = len(arms["left"].sent)
        time.sleep(0.2)
        assert store.read().sim_q["left"] == before
        assert len(arms["left"].sent) > sent_before


def test_start_without_connected_arm_returns_to_teleop():
    with _controller() as (controller, store, _arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("STOP"))
        _wait(lambda: store.read().phase == "STOPPED")
        _wait(lambda: store.read().can_reset)
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")


def test_snapshot_reports_tightest_limit_margin():
    with _controller() as (controller, store, _arms):
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: store.read().joint_margin_deg is not None)
        margin = store.read().joint_margin_deg
        assert margin is not None and margin >= 0.0


def test_dead_can_link_disconnects_arm_and_latches_stop():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: bool(arms["left"].sent))
        arms["left"].transport_dead = "CAN transport on can0 closed"
        _wait(lambda: store.read().arm_status["left"].state == "ERROR")
        _wait(lambda: store.read().phase == "STOPPED")
        assert "CAN transport" in (store.read().arm_status["left"].error or "")
        _wait(lambda: arms["left"].closed)
        assert store.read().real_q["left"] is None
