from __future__ import annotations

import logging

import numpy as np

from config import RosOutputConfig
from transforms import quat_xyzw_from_rotation_matrix

SIDES = ("left", "right")
TOPICS = {
    "current_joints": "current_joint_angles_deg",
    "target_joints": "target_joint_angles_deg",
    "current_pose": "current_end_effector_pose",
    "target_pose": "target_end_effector_pose",
}


class RosOutput:
    """Optional ROS 2 publisher for /nero/{left,right} telemetry."""

    def __init__(self, config: RosOutputConfig, logger: logging.Logger) -> None:
        self.config = config
        self.logger = logger
        self.enabled = False
        self.available: bool | None = None
        self.reason = ""
        self.node = None
        self._rclpy = None
        self._PoseStamped = None
        self._publishers: dict[str, object] = {}

    def set_enabled(self, enabled: bool) -> bool:
        if enabled and not self._publishers:
            if not self._initialize():
                return False
        self.enabled = bool(enabled)
        return True

    def _initialize(self) -> bool:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import Float64MultiArray

            if not rclpy.ok():
                rclpy.init(args=[])
            self._rclpy = rclpy
            self._PoseStamped = PoseStamped
            self._Float64MultiArray = Float64MultiArray
            self.node = rclpy.create_node("nero_output")
            prefix = self.config.prefix.rstrip("/")
            for side in SIDES:
                base = f"{prefix}/{side}"
                self._publishers[f"{side}_current_joints"] = self.node.create_publisher(
                    Float64MultiArray, f"{base}/{TOPICS['current_joints']}", qos_profile_sensor_data
                )
                self._publishers[f"{side}_target_joints"] = self.node.create_publisher(
                    Float64MultiArray, f"{base}/{TOPICS['target_joints']}", qos_profile_sensor_data
                )
                self._publishers[f"{side}_current_pose"] = self.node.create_publisher(
                    PoseStamped, f"{base}/{TOPICS['current_pose']}", qos_profile_sensor_data
                )
                self._publishers[f"{side}_target_pose"] = self.node.create_publisher(
                    PoseStamped, f"{base}/{TOPICS['target_pose']}", qos_profile_sensor_data
                )
            self.available = True
            self.reason = ""
            self.logger.info("ROS 2 output ready: %s/{left,right}/*", prefix)
            return True
        except Exception as exc:
            self.available = False
            self.reason = str(exc)
            self.enabled = False
            self.logger.warning("ROS 2 output unavailable: %s", exc)
            return False

    def publish(
        self,
        side: str,
        current_q: np.ndarray | None,
        target_q: np.ndarray | None,
        current_pose: np.ndarray | None,
        target_pose: np.ndarray | None,
    ) -> None:
        if not self.enabled or self.node is None:
            return
        if current_q is not None:
            self._joints(f"{side}_current_joints", current_q)
        if target_q is not None:
            self._joints(f"{side}_target_joints", target_q)
        if current_pose is not None:
            self._pose(f"{side}_current_pose", current_pose)
        if target_pose is not None:
            self._pose(f"{side}_target_pose", target_pose)

    def _joints(self, key: str, q: np.ndarray) -> None:
        values = np.rad2deg(np.asarray(q, dtype=float).reshape(7))
        self._publishers[key].publish(self._Float64MultiArray(data=[float(v) for v in values]))

    def _pose(self, key: str, transform: np.ndarray) -> None:
        transform = np.asarray(transform, dtype=float).reshape(4, 4)
        quat = quat_xyzw_from_rotation_matrix(transform[:3, :3])
        msg = self._PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.config.frame_id
        msg.pose.position.x = float(transform[0, 3])
        msg.pose.position.y = float(transform[1, 3])
        msg.pose.position.z = float(transform[2, 3])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._publishers[key].publish(msg)

    def close(self) -> None:
        self.enabled = False
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self.node = None
        if self._rclpy is not None and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
        self._rclpy = None
        self._publishers.clear()
