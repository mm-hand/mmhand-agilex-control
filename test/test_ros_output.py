from __future__ import annotations

import logging
from types import SimpleNamespace

import numpy as np

from config import RosOutputConfig
from ros_output import TOPICS, RosOutput


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeHeader:
    def __init__(self) -> None:
        self.stamp = None
        self.frame_id = ""


class FakePoseStamped:
    def __init__(self) -> None:
        self.header = FakeHeader()
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
        )


class FakeFloat64MultiArray:
    def __init__(self, data) -> None:
        self.data = data


def _output() -> RosOutput:
    ros = RosOutput(RosOutputConfig(), logging.getLogger("test"))
    ros.enabled = True
    ros.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp")))
    ros._PoseStamped = FakePoseStamped
    ros._Float64MultiArray = FakeFloat64MultiArray
    ros._publishers = {
        f"{side}_{key}": FakePublisher() for side in ("left", "right") for key in TOPICS
    }
    return ros


def test_ros_output_publishes_degrees_and_pose():
    ros = _output()
    q = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    pose = np.eye(4)
    ros.publish("left", q, q, pose, pose)
    joints = ros._publishers["left_current_joints"].messages[0]
    assert np.allclose(joints.data, np.rad2deg(q))
    assert ros._publishers["left_target_joints"].messages
    msg = ros._publishers["left_current_pose"].messages[0]
    assert msg.header.frame_id == "pose_input_horizon"
    assert msg.pose.orientation.w == 1.0


def test_ros_output_does_nothing_when_disabled():
    ros = _output()
    ros.enabled = False
    ros.publish("left", np.zeros(7), np.zeros(7), np.eye(4), np.eye(4))
    assert not ros._publishers["left_current_joints"].messages


def test_ros_topic_names_are_nero_side_names():
    assert TOPICS["current_joints"] == "current_joint_angles_deg"
    assert TOPICS["target_joints"] == "target_joint_angles_deg"
    assert TOPICS["current_pose"] == "current_end_effector_pose"
    assert TOPICS["target_pose"] == "target_end_effector_pose"
