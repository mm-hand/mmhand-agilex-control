from __future__ import annotations
import time
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from config import PoseInputConfig
from frames import ControllerPose
from transforms import quat_xyzw_from_rotation_matrix

JOINT_NAMES = [f"joint{joint}" for joint in range(1, 8)]


def _stamp_from_seconds(seconds: float) -> Time:
    if not np.isfinite(seconds) or seconds <= 0.0:
        raise ValueError("SDK timestamp must be finite and positive")
    sec = int(seconds)
    nanosec = round((float(seconds) - sec) * 1_000_000_000)
    if nanosec == 1_000_000_000:
        sec, nanosec = sec + 1, 0
    return Time(sec=sec, nanosec=nanosec)


class PoseInputSubscriber:
    def __init__(self, config: PoseInputConfig) -> None:
        self.context = rclpy.Context()
        self.context.init()
        self.config = config
        self.node = rclpy.create_node(config.node_name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.tracked = (0, 0)
        self.left: ControllerPose | None = None
        self.right: ControllerPose | None = None
        self.last_poses_at: float = 0.0
        self.node.create_subscription(
            PoseArray, config.poses_topic, self._on_poses, qos_profile_sensor_data
        )
        self.node.create_subscription(
            UInt8MultiArray, config.tracked_topic, self._on_tracked, qos_profile_sensor_data
        )
        self.left_current_joints = self.node.create_publisher(JointState, "/analytic_control/left/current_joints", qos_profile_sensor_data)
        self.left_target_joints = self.node.create_publisher(JointState, "/analytic_control/left/target_joints", qos_profile_sensor_data)
        self.left_current_pose = self.node.create_publisher(PoseStamped, "/analytic_control/left/current_pose", qos_profile_sensor_data)
        self.left_target_pose = self.node.create_publisher(PoseStamped, "/analytic_control/left/target_pose", qos_profile_sensor_data)
        self.right_current_joints = self.node.create_publisher(JointState, "/analytic_control/right/current_joints", qos_profile_sensor_data)
        self.right_target_joints = self.node.create_publisher(JointState, "/analytic_control/right/target_joints", qos_profile_sensor_data)
        self.right_current_pose = self.node.create_publisher(PoseStamped, "/analytic_control/right/current_pose", qos_profile_sensor_data)
        self.right_target_pose = self.node.create_publisher(PoseStamped, "/analytic_control/right/target_pose", qos_profile_sensor_data)
    def _on_poses(self, msg: PoseArray) -> None:
        now = time.monotonic()
        self.last_poses_at = now
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
    def connected(self, stale_s: float) -> bool:
        return self.last_poses_at > 0.0 and time.monotonic() - self.last_poses_at < stale_s
    def _fresh(self, pose: ControllerPose | None) -> ControllerPose | None:
        if pose is None or time.monotonic() - pose.received_at > self.config.max_pose_age_s:
            return None
        return pose
    def get_left(self) -> ControllerPose | None:
        return self._fresh(self.left)
    def get_right(self) -> ControllerPose | None:
        return self._fresh(self.right)
    def publish_telemetry(
        self,
        left_current_q: np.ndarray | None,
        left_target_q: np.ndarray | None,
        left_current_pose: np.ndarray | None,
        left_target_pose: np.ndarray | None,
        right_current_q: np.ndarray | None,
        right_target_q: np.ndarray | None,
        right_current_pose: np.ndarray | None,
        right_target_pose: np.ndarray | None,
        *,
        left_current_timestamp_s: float | None = None,
        right_current_timestamp_s: float | None = None,
    ) -> None:
        if not self.context.ok():
            return
        target_stamp = self.node.get_clock().now().to_msg()
        left_current_stamp = (
            target_stamp if left_current_timestamp_s is None
            else _stamp_from_seconds(left_current_timestamp_s)
        )
        right_current_stamp = (
            target_stamp if right_current_timestamp_s is None
            else _stamp_from_seconds(right_current_timestamp_s)
        )
        self._publish_joints(self.left_current_joints, left_current_q, left_current_stamp)
        self._publish_joints(self.left_target_joints, left_target_q, target_stamp)
        self._publish_pose(self.left_current_pose, left_current_pose, left_current_stamp)
        self._publish_pose(self.left_target_pose, left_target_pose, target_stamp)
        self._publish_joints(self.right_current_joints, right_current_q, right_current_stamp)
        self._publish_joints(self.right_target_joints, right_target_q, target_stamp)
        self._publish_pose(self.right_current_pose, right_current_pose, right_current_stamp)
        self._publish_pose(self.right_target_pose, right_target_pose, target_stamp)
    def _publish_joints(self, publisher, q: np.ndarray | None, stamp: Time) -> None:
        if q is None:
            return
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in np.asarray(q, dtype=float).reshape(7)]
        publisher.publish(msg)
    def _publish_pose(self, publisher, transform: np.ndarray | None, stamp) -> None:
        if transform is None:
            return
        transform = np.asarray(transform, dtype=float).reshape(4, 4)
        quat = quat_xyzw_from_rotation_matrix(transform[:3, :3])
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "pose_input_horizon"
        msg.pose.position.x = float(transform[0, 3])
        msg.pose.position.y = float(transform[1, 3])
        msg.pose.position.z = float(transform[2, 3])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        publisher.publish(msg)
    def close(self) -> None:
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()
