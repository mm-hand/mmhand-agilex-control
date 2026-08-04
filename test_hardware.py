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
        self.q = np.zeros(7)
        self.timestamp = 0.0
        self.diagnostic_reads = 0

    def get_joint_angles(self):
        self.timestamp += 1.0
        return SimpleNamespace(msg=self.q.copy(), timestamp=self.timestamp)
    def get_motor_states(self, joint):
        self.diagnostic_reads += 1
        return SimpleNamespace(msg=SimpleNamespace(
            position=0.1 * joint, velocity=0.0, current=float(joint), torque=0.0
        ))

    def get_driver_states(self, joint):
        self.diagnostic_reads += 1
        return SimpleNamespace(msg=SimpleNamespace(
            vol=48.0, foc_temp=30.0, motor_temp=31.0, bus_current=0.2,
            foc_status=SimpleNamespace(driver_enable_status=True),
        ))

    def get_arm_status(self):
        self.diagnostic_reads += 1
        return SimpleNamespace(msg=SimpleNamespace(
            ctrl_mode=1, arm_status=0, mode_feedback=6, motion_status=0,
            err_status=SimpleNamespace(),
        ))


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
    cfg = RealArmConfig(firmware="v120", startup_timeout_s=0.1, servo_handoff_s=0.0)
    backend = RealArmBackend(
        side,
        cfg,
        MotionConfig(command_hz=1000.0),
        SimpleNamespace(info=lambda *args, **kwargs: None),
        robot_factory=lambda _cfg, _logger: (robot, "v120"),
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
    assert robot.sent_js[-1] == [0.1] * 7
    backend.close()
    assert robot.disconnected

def test_servo_handoff_holds_fresh_feedback():
    backend, robot = _backend("left")
    startup = np.linspace(0.1, 0.7, 7)
    robot.q = startup.copy()
    hold = backend.prepare_teleop("left")
    assert np.allclose(hold, startup)
    assert robot.motion_modes == ["js"]
    assert np.allclose(robot.sent_js[0], startup)

def test_startup_move_refreshes_feedback():
    backend, robot = _backend("left")
    target = np.full(7, 0.001)
    backend.move_j_startup("left", target)
    assert robot.motion_modes == ["j"]
    assert robot.sent_j
    assert np.allclose(backend.q_feedback, target, atol=1e-6)

def test_real_backend_right_side():
    backend, robot = _backend("right")
    frames = CoordinateFrames(left=None, right=None)
    backend.update(np.ones(7) * 0.1, np.ones(7) * 0.2, frames, "ok", 0.02)
    assert robot.sent_js == [[0.2] * 7]
    backend.close()


def test_diagnostics_are_rate_limited():
    backend, robot = _backend("left")
    diagnostics = backend.diagnostics()
    assert diagnostics["motors"][0]["current"] == 1.0
    assert robot.diagnostic_reads == 15
    assert backend.diagnostics() is None
    assert robot.diagnostic_reads == 15


if __name__ == "__main__":
    test_validate_joint_command_rejects_nan()
    test_real_backend_sends_only_active_side()
    test_servo_handoff_holds_fresh_feedback()
    test_startup_move_refreshes_feedback()
    test_real_backend_right_side()
    test_diagnostics_are_rate_limited()
    print("All hardware tests passed.")
