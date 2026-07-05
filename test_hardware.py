from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from config import MotionConfig, RealArmConfig
from frames import CoordinateFrames
from hardware import RealArmBackend, validate_joint_command


class FakeRobot:
    def __init__(self) -> None:
        self.sent_js = []
        self.sent_j = []
        self.disconnected = False
        self.motion_modes = []
        self.auto_modes = []
        self.normal_count = 0

    def get_joint_angles(self):
        return SimpleNamespace(msg=[0.0] * 7)

    def set_motion_mode(self, mode):
        self.motion_modes.append(mode)

    def set_auto_set_motion_mode_enabled(self, enabled):
        self.auto_modes.append(bool(enabled))

    def move_js(self, joints):
        self.sent_js.append(list(joints))

    def move_j(self, joints):
        self.sent_j.append(list(joints))

    def set_normal_mode(self):
        self.normal_count += 1

    def disconnect(self):
        self.disconnected = True


def _backend(side="left"):
    robot = FakeRobot()
    cfg = RealArmConfig(move_to_startup_pose=False, require_enter_before_startup_move=False, firmware="v120")
    backend = RealArmBackend(
        side,
        cfg,
        MotionConfig(),
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


if __name__ == "__main__":
    test_validate_joint_command_rejects_nan()
    test_real_backend_sends_only_active_side()
    test_real_backend_right_side()
    print("All hardware tests passed.")
