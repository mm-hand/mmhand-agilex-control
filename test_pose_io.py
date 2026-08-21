from types import SimpleNamespace

import numpy as np

from pose_io import PoseInputSubscriber


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _publisher() -> PoseInputSubscriber:
    publisher = PoseInputSubscriber.__new__(PoseInputSubscriber)
    publisher.context = SimpleNamespace(ok=lambda: True)
    stamp = SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=1, nanosec=2))
    publisher.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: stamp))
    for side in ("left", "right"):
        for state in ("current", "target"):
            setattr(publisher, f"{side}_{state}_joints", FakePublisher())
            setattr(publisher, f"{side}_{state}_pose", FakePublisher())
    return publisher


def test_publish_telemetry_updates_all_eight_existing_topics():
    publisher = _publisher()
    q = np.arange(7, dtype=float)
    pose = np.eye(4)
    publisher.publish_telemetry(q, q, pose, pose, q, q, pose, pose)
    for side in ("left", "right"):
        for state in ("current", "target"):
            joints = getattr(publisher, f"{side}_{state}_joints").messages
            poses = getattr(publisher, f"{side}_{state}_pose").messages
            assert len(joints) == 1
            assert list(joints[0].data) == q.tolist()
            assert len(poses) == 1
            assert poses[0].header.frame_id == "pose_input_horizon"


def test_publish_telemetry_preserves_missing_side_semantics():
    publisher = _publisher()
    q = np.zeros(7)
    pose = np.eye(4)
    publisher.publish_telemetry(q, q, pose, pose, None, None, None, None)
    assert len(publisher.left_current_joints.messages) == 1
    assert len(publisher.left_target_pose.messages) == 1
    assert not publisher.right_current_joints.messages
    assert not publisher.right_target_pose.messages
