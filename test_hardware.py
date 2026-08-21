from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from config import MotionConfig, RealArmConfig
from frames import CoordinateFrames
from hardware import (
    RealArmBackend,
    _firmware_name,
    _socketcan_state,
    detect_firmware,
    validate_joint_command,
)


class FakeRobot:
    def __init__(self) -> None:
        self.sent_js = []
        self.sent_j = []
        self.disconnected = False
        self.motion_modes = []
        self.auto_modes = []
        self.normal_count = 0
        self.q = np.zeros(7)
        self.motor_q = np.zeros(7)
        self.timestamp = 1.0
        self.hz = 100.0
        self.motor_available = True
        self.diagnostic_reads = 0

    # Nero 1.10 may keep this timestamp static while updating msg.
    def get_joint_angles(self):
        return SimpleNamespace(msg=self.q.copy(), timestamp=self.timestamp)
    def get_motor_states(self, joint):
        self.diagnostic_reads += 1
        if not self.motor_available:
            return None
        return SimpleNamespace(msg=SimpleNamespace(
            position=self.motor_q[joint - 1], velocity=0.0, current=float(joint), torque=0.0
        ), timestamp=self.timestamp, hz=self.hz)

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
        self.motor_q = np.asarray(joints, dtype=float)
        self.timestamp += 0.01

    def move_j(self, joints):
        self.sent_j.append(list(joints))
        self.q = np.asarray(joints, dtype=float)
        self.motor_q = np.asarray(joints, dtype=float)
        self.timestamp += 0.01

    def set_normal_mode(self):
        self.normal_count += 1

    def disconnect(self):
        self.disconnected = True


class FakeDetectionRobot:
    def __init__(self, firmware=None) -> None:
        self.firmware = firmware
        self.enable_count = 0
        self.disconnected = False

    def get_firmware(self, timeout, min_interval):
        return self.firmware

    def enable(self):
        self.enable_count += 1
        return False

    def disconnect(self):
        self.disconnected = True


def _backend(side="left"):
    robot = FakeRobot()
    cfg = RealArmConfig(firmware="v120", startup_timeout_s=0.1, servo_handoff_s=0.0)
    backend = RealArmBackend(
        side,
        cfg,
        MotionConfig(command_hz=1000.0),
        SimpleNamespace(info=lambda *args, **kwargs: None, warning=lambda *args, **kwargs: None),
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


def test_firmware_name_accepts_can_and_web_versions():
    expected = {
        "1.10": "default",
        "1.11": "v111",
        "1.12": "v112",
        "1.20": "v120",
        "1.21": "v120",
        "V1.2.1": "v120",
    }
    assert {version: _firmware_name(version) for version in expected} == expected


def test_firmware_name_rejects_unknown_formats():
    for version in ("", "1", "1.2.10", "sdk-1.0.0"):
        try:
            _firmware_name(version)
        except RuntimeError:
            continue
        raise AssertionError(f"invalid firmware version accepted: {version!r}")


def test_socketcan_state_reads_ip_json():
    output = '[{"linkinfo":{"info_data":{"state":"ERROR-ACTIVE"}}}]'
    with patch("hardware.subprocess.run", return_value=SimpleNamespace(stdout=output)):
        assert _socketcan_state(RealArmConfig()) == "ERROR-ACTIVE"


def test_socketcan_state_fails_closed_when_ip_cannot_be_read():
    with patch("hardware.subprocess.run", side_effect=FileNotFoundError):
        try:
            _socketcan_state(RealArmConfig())
        except RuntimeError as exc:
            assert "failed to inspect SocketCAN" in str(exc)
        else:
            raise AssertionError("SocketCAN inspection failure should stop startup")


def test_detect_firmware_selects_v120_without_enable_when_query_succeeds():
    robot = FakeDetectionRobot({"software_version": "1.21"})
    config = RealArmConfig(interface="test", enable_timeout_s=0.1)
    logger = SimpleNamespace(info=lambda *args: None)
    with (
        patch("hardware.create_robot", return_value=robot),
        patch("hardware.install_can_send_timeout") as install_timeout,
    ):
        assert detect_firmware(config, logger) == "v120"
    install_timeout.assert_called_once_with(robot, config.can_send_timeout_s)
    assert robot.enable_count == 0
    assert robot.disconnected


def test_detect_firmware_rejects_invalid_reported_version():
    robot = FakeDetectionRobot({"software_version": "unknown"})
    config = RealArmConfig(interface="test", enable_timeout_s=0.1)
    logger = SimpleNamespace(info=lambda *args: None)
    with (
        patch("hardware.create_robot", return_value=robot),
        patch("hardware.install_can_send_timeout"),
    ):
        try:
            detect_firmware(config, logger)
        except RuntimeError as exc:
            assert "invalid Nero firmware version" in str(exc)
        else:
            raise AssertionError("invalid reported firmware should stop detection")
    assert robot.enable_count == 0
    assert robot.disconnected


def test_detect_firmware_limits_compatibility_enable_rate():
    robot = FakeDetectionRobot()
    config = RealArmConfig(interface="test", enable_timeout_s=0.12)
    logger = SimpleNamespace(info=lambda *args: None)
    with (
        patch("hardware.create_robot", return_value=robot),
        patch("hardware.install_can_send_timeout"),
    ):
        try:
            detect_firmware(config, logger)
        except RuntimeError:
            pass
        else:
            raise AssertionError("firmware detection should time out")
    assert robot.enable_count == 1
    assert robot.disconnected


def test_detect_firmware_stops_on_socketcan_fault():
    robot = FakeDetectionRobot()
    config = RealArmConfig(enable_timeout_s=1.0)
    logger = SimpleNamespace(info=lambda *args: None)
    with (
        patch("hardware.create_robot", return_value=robot),
        patch("hardware.install_can_send_timeout"),
        patch("hardware._socketcan_state", return_value="ERROR-PASSIVE"),
    ):
        try:
            detect_firmware(config, logger)
        except RuntimeError as exc:
            assert "ERROR-PASSIVE" in str(exc)
        else:
            raise AssertionError("CAN fault should stop firmware detection")
    assert robot.enable_count == 0
    assert robot.disconnected


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


def test_servo_handoff_uses_last_command_when_feedback_is_stale():
    backend, robot = _backend("left")
    expected = np.linspace(0.1, 0.7, 7)
    backend.last_sent_q = expected.copy()
    robot.q = np.full(7, 2.0)
    robot.motor_q = robot.q.copy()
    backend.joint_updated_at -= backend.config.feedback_stale_s + 0.01
    backend.motor_updated_at -= backend.config.feedback_stale_s + 0.01
    hold = backend.prepare_teleop("left")
    assert np.allclose(hold, expected)
    assert np.allclose(robot.sent_js[0], expected)


def test_servo_handoff_returns_the_command_it_holds():
    backend, robot = _backend("left")
    expected = np.linspace(0.1, 0.7, 7)
    observed = np.linspace(-0.7, -0.1, 7)
    robot.q = expected.copy()

    def move_js(joints):
        robot.sent_js.append(list(joints))
        robot.q = observed.copy()
        robot.motor_q = observed.copy()
        robot.timestamp += 0.01

    robot.move_js = move_js
    hold = backend.prepare_teleop("left")
    assert np.allclose(robot.sent_js[0], expected)
    assert np.allclose(hold, expected)


def test_startup_move_refreshes_feedback():
    backend, robot = _backend("left")
    target = np.full(7, 0.001)
    backend.move_j_startup("left", target)
    assert robot.motion_modes == ["j"]
    assert robot.sent_j
    assert np.allclose(backend.q_feedback, target, atol=1e-6)

def test_startup_does_not_require_reaching_target_or_fresh_timestamp():
    backend, robot = _backend("left")
    actual = np.full(7, 0.0005)
    robot.move_j = lambda joints: robot.sent_j.append(list(joints))
    robot.q = actual
    robot.motor_q = actual
    backend.move_j_startup("left", np.full(7, 0.001))
    assert np.allclose(backend.q_feedback, actual)

def test_real_backend_right_side():
    backend, robot = _backend("right")
    frames = CoordinateFrames(left=None, right=None)
    backend.update(np.ones(7) * 0.1, np.ones(7) * 0.2, frames, "ok", 0.02)
    assert robot.sent_js == [[0.2] * 7]
    backend.close()


def test_diagnostics_are_rate_limited():
    backend, robot = _backend("left")
    robot.diagnostic_reads = 0
    diagnostics = backend.diagnostics()
    assert diagnostics["motors"][0]["current"] == 1.0
    assert robot.diagnostic_reads == 15
    assert backend.diagnostics() is None
    assert robot.diagnostic_reads == 15


def test_current_feedback_uses_get_joint_angles():
    backend, robot = _backend("left")
    robot.q = np.full(7, -1.0)
    robot.motor_q = np.arange(7, dtype=float) * 0.1
    robot.timestamp += 0.01
    backend.refresh_feedback()
    assert np.allclose(backend.current_joints("left"), robot.q)


def test_stale_motor_feedback_keeps_last_valid_angles():
    backend, robot = _backend("left")
    valid = np.arange(7, dtype=float) * 0.1
    robot.q = valid
    robot.motor_q = valid
    robot.timestamp += 0.01
    backend.refresh_feedback()
    robot.q = np.full(7, 2.0)
    robot.motor_q = np.full(7, 2.0)
    backend.joint_updated_at -= backend.config.feedback_stale_s + 0.01
    backend.motor_updated_at -= backend.config.feedback_stale_s + 0.01
    backend.refresh_feedback()
    assert np.allclose(backend.current_joints("left"), valid)
    backend.update(np.full(7, 0.2), None, CoordinateFrames(left=None, right=None), "ok", 0.02)
    assert np.allclose(robot.sent_js[-1], 0.2)


if __name__ == "__main__":
    test_validate_joint_command_rejects_nan()
    test_real_backend_sends_only_active_side()
    test_servo_handoff_holds_fresh_feedback()
    test_servo_handoff_uses_last_command_when_feedback_is_stale()
    test_servo_handoff_returns_the_command_it_holds()
    test_startup_move_refreshes_feedback()
    test_startup_does_not_require_reaching_target_or_fresh_timestamp()
    test_real_backend_right_side()
    test_diagnostics_are_rate_limited()
    test_current_feedback_uses_get_joint_angles()
    test_stale_motor_feedback_keeps_last_valid_angles()
    print("All hardware tests passed.")
