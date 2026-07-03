from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8MultiArray

from config import RosConfig
from frames import ControllerPose


class PicoPoseSubscriber:
    def __init__(self, config: RosConfig) -> None:
        self.context = rclpy.Context()
        self.context.init()
        self.config = config
        self.node = rclpy.create_node(config.node_name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.tracked = (0, 0)
        self.left: ControllerPose | None = None
        self.right: ControllerPose | None = None
        self.node.create_subscription(
            PoseArray, config.poses_topic, self._on_poses, qos_profile_sensor_data
        )
        self.node.create_subscription(
            UInt8MultiArray, config.tracked_topic, self._on_tracked, qos_profile_sensor_data
        )

    def _on_poses(self, msg: PoseArray) -> None:
        now = time.monotonic()
        n = len(msg.poses)
        self.left = (
            ControllerPose.from_ros_pose(msg.poses[0], now)
            if n >= 1 and self.tracked[0] == 1
            else None
        )
        self.right = (
            ControllerPose.from_ros_pose(msg.poses[1], now)
            if n >= 2 and self.tracked[1] == 1
            else None
        )

    def _on_tracked(self, msg: UInt8MultiArray) -> None:
        data = list(msg.data)
        left = int(data[0]) if len(data) >= 1 else 0
        right = int(data[1]) if len(data) >= 2 else left
        self.tracked = (left, right)

    def spin(self) -> None:
        self.executor.spin_once(timeout_sec=0)

    def _fresh(self, pose: ControllerPose | None) -> ControllerPose | None:
        if pose is None or time.monotonic() - pose.received_at > self.config.max_pose_age_s:
            return None
        return pose

    def get_left(self) -> ControllerPose | None:
        return self._fresh(self.left)

    def get_right(self) -> ControllerPose | None:
        return self._fresh(self.right)

    def close(self) -> None:
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()
