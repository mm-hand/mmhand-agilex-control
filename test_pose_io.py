from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from pose_io import JOINT_NAMES, PoseInputSubscriber, _stamp_from_seconds


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _publisher() -> PoseInputSubscriber:
    publisher = PoseInputSubscriber.__new__(PoseInputSubscriber)
    publisher.context = SimpleNamespace(ok=lambda: True)
    stamp = SimpleNamespace(to_msg=lambda: Time(sec=1, nanosec=2))
    publisher.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: stamp))
    for side in ("left", "right"):
        for state in ("current", "target"):
            setattr(publisher, f"{side}_{state}_joints", FakePublisher())
            setattr(publisher, f"{side}_{state}_pose", FakePublisher())
    return publisher


def test_publish_telemetry_stamps_all_topics_from_the_correct_clock():
    publisher = _publisher()
    q = np.arange(7, dtype=float)
    pose = np.eye(4)
    publisher.publish_telemetry(
        q, q, pose, pose, q, q, pose, pose,
        left_current_timestamp_s=10.25,
        right_current_timestamp_s=20.5,
    )
    for side in ("left", "right"):
        for state in ("current", "target"):
            joints = getattr(publisher, f"{side}_{state}_joints").messages
            poses = getattr(publisher, f"{side}_{state}_pose").messages
            assert len(joints) == 1
            assert list(joints[0].name) == JOINT_NAMES
            assert list(joints[0].position) == q.tolist()
            expected = {
                ("left", "current"): (10, 250_000_000),
                ("right", "current"): (20, 500_000_000),
            }.get((side, state), (1, 2))
            assert (joints[0].header.stamp.sec, joints[0].header.stamp.nanosec) == expected
            assert len(poses) == 1
            assert poses[0].header.frame_id == "pose_input_horizon"
            assert (poses[0].header.stamp.sec, poses[0].header.stamp.nanosec) == expected


def test_publish_telemetry_preserves_missing_side_semantics():
    publisher = _publisher()
    q = np.zeros(7)
    pose = np.eye(4)
    publisher.publish_telemetry(q, q, pose, pose, None, None, None, None)
    assert len(publisher.left_current_joints.messages) == 1
    assert len(publisher.left_target_pose.messages) == 1
    assert not publisher.right_current_joints.messages
    assert not publisher.right_target_pose.messages


def test_sim_telemetry_uses_host_stamp_for_current_and_target():
    publisher = _publisher()
    q = np.zeros(7)
    pose = np.eye(4)
    publisher.publish_telemetry(q, q, pose, pose, None, None, None, None)
    assert publisher.left_current_joints.messages[0].header.stamp == Time(sec=1, nanosec=2)
    assert publisher.left_target_joints.messages[0].header.stamp == Time(sec=1, nanosec=2)


def test_sdk_timestamp_conversion_normalizes_nanosecond_carry():
    stamp = _stamp_from_seconds(10.9999999996)
    assert (stamp.sec, stamp.nanosec) == (11, 0)
    epoch_seconds = 1_750_000_000.1234567
    epoch_stamp = _stamp_from_seconds(epoch_seconds)
    assert abs(epoch_stamp.sec + epoch_stamp.nanosec / 1e9 - epoch_seconds) < 0.5e-6
    for invalid in (0.0, -1.0, np.nan, np.inf):
        with pytest.raises(ValueError):
            _stamp_from_seconds(invalid)
