from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from config import ControlConfig, MotionConfig, RealArmConfig
from control_loop import _run_startup
from frames import CoordinateFrames
from hardware import RealArmBackend, validate_joint_command
from sim import SwiftSimBackend, _comparison_error


class FakeRobot:
    def __init__(self) -> None:
        self.q = np.zeros(7)
        self.sent_js = []
        self.sent_j = []
        self.disconnected = False
        self.motion_modes = []
        self.auto_modes = []
        self.normal_count = 0

    def get_joint_angles(self):
        return SimpleNamespace(msg=self.q.tolist())

    def set_motion_mode(self, mode):
        self.motion_modes.append(mode)

    def set_auto_set_motion_mode_enabled(self, enabled):
        self.auto_modes.append(bool(enabled))

    def move_js(self, joints):
        self.sent_js.append(list(joints))
        self.q = np.asarray(joints, dtype=float)

    def move_j(self, joints):
        self.sent_j.append(list(joints))
        self.q = np.asarray(joints, dtype=float)

    def set_normal_mode(self):
        self.normal_count += 1

    def disconnect(self):
        self.disconnected = True


def _backend(side="left"):
    robot = FakeRobot()
    cfg = RealArmConfig(firmware="v120")
    backend = RealArmBackend(
        side,
        cfg,
        MotionConfig(command_hz=1000.0),
        SimpleNamespace(info=lambda *args, **kwargs: None),
        robot_factory=lambda _cfg, _logger: robot,
    )
    return backend, robot


def test_validate_joint_command_rejects_nan():
    q = np.zeros(7)
    q[0] = np.nan
    try:
        validate_joint_command(q)
    except ValueError:
        return
    raise AssertionError("NaN command was not rejected")


def test_real_backend_sends_only_active_side():
    backend, robot = _backend("left")
    frames = CoordinateFrames(left=None, right=None)
    backend.update(np.ones(7) * 0.1, np.ones(7) * 0.2, frames, "ok", 0.02)
    assert robot.sent_js == [[0.1] * 7]
    backend.update(None, np.ones(7) * 0.2, frames, "ok", 0.02)
    assert len(robot.sent_js) == 1
    backend.close()
    assert robot.disconnected


def test_real_backend_right_side():
    backend, robot = _backend("right")
    frames = CoordinateFrames(left=None, right=None)
    backend.update(np.ones(7) * 0.1, np.ones(7) * 0.2, frames, "ok", 0.02)
    assert robot.sent_js == [[0.2] * 7]
    backend.close()


def test_real_backend_refreshes_feedback():
    backend, robot = _backend("left")
    robot.q = np.arange(7, dtype=float) * 0.1
    assert np.allclose(backend.current_joints(), robot.q)


def test_comparison_pose_error():
    real = np.eye(4)
    sim = np.eye(4)
    assert _comparison_error(real, sim) == (0.0, 0.0)
    sim[0, 3] = 0.1
    sim[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    position, rotation = _comparison_error(real, sim)
    assert np.isclose(position, 0.1)
    assert np.isclose(rotation, 90.0)


def test_real_backend_startup_uses_move_j_then_move_js():
    backend, robot = _backend("left")
    target = np.full(7, 0.001)
    backend.move_j_startup("left", target)
    assert robot.sent_j
    assert robot.motion_modes[-2:] == ["j", "js"]
    assert np.allclose(backend.q_feedback, target)


def test_sim_backend_follows_startup_trajectory():
    backend = SwiftSimBackend.__new__(SwiftSimBackend)
    backend.left_robot = SimpleNamespace(q=np.zeros(7))
    backend.right_robot = SimpleNamespace(q=np.zeros(7))
    backend.motion_config = MotionConfig(command_hz=1000.0)
    backend.status_label = SimpleNamespace(desc="")
    steps = []
    backend.env = SimpleNamespace(step=lambda dt: steps.append(dt))

    target = np.full(7, 0.001)
    backend.move_j_startup("left", target)
    assert steps
    assert np.allclose(backend.left_robot.q, target)
    assert np.allclose(backend.right_robot.q, 0.0)


def test_startup_moves_active_arms_before_enter():
    config = ControlConfig()
    events = []

    class Backend:
        def move_j_startup(self, side, q):
            events.append((side, np.asarray(q, dtype=float)))

    _run_startup(config, Backend(), True, True, lambda prompt: events.append(("enter", prompt)))
    assert [event[0] for event in events] == ["left", "right", "enter"]
    assert np.allclose(events[0][1], config.startup.left_pose_rad)
    assert np.allclose(events[1][1], config.startup.right_pose_rad)

    events.clear()
    disabled = ControlConfig(startup=replace(config.startup, enable_startup_pose=False))
    _run_startup(disabled, Backend(), True, True, lambda prompt: events.append(("enter", prompt)))
    assert events == []


if __name__ == "__main__":
    test_validate_joint_command_rejects_nan()
    test_real_backend_sends_only_active_side()
    test_real_backend_right_side()
    test_real_backend_refreshes_feedback()
    test_comparison_pose_error()
    test_real_backend_startup_uses_move_j_then_move_js()
    test_sim_backend_follows_startup_trajectory()
    test_startup_moves_active_arms_before_enter()
    print("All hardware tests passed.")
