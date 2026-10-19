from __future__ import annotations

import logging


class PicoRosOutput:
    """Optional /read_pose/* publisher for the PICO bridge process."""

    def __init__(self, logger: logging.Logger) -> None:
        self.logger = logger
        self.enabled = False
        self.available: bool | None = None
        self.reason = ""
        self.node = None
        self._rclpy = None
        self._poses = None
        self._buttons = None
        self._tracked = None

    def set_enabled(self, enabled: bool) -> bool:
        if enabled and self.node is None:
            if not self._initialize():
                return False
        self.enabled = bool(enabled)
        return True

    def _initialize(self) -> bool:
        try:
            import rclpy
            from geometry_msgs.msg import Pose, PoseArray
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import UInt8MultiArray, UInt32MultiArray

            if not rclpy.ok():
                rclpy.init(args=[])
            self._rclpy = rclpy
            self._Pose = Pose
            self._PoseArray = PoseArray
            self._UInt8 = UInt8MultiArray
            self._UInt32 = UInt32MultiArray
            self.node = rclpy.create_node("read_pose")
            self._poses = self.node.create_publisher(PoseArray, "/read_pose/poses", qos_profile_sensor_data)
            self._buttons = self.node.create_publisher(UInt32MultiArray, "/read_pose/buttons", qos_profile_sensor_data)
            self._tracked = self.node.create_publisher(UInt8MultiArray, "/read_pose/tracked", qos_profile_sensor_data)
            self.available = True
            self.reason = ""
            self.logger.info("ROS 2 input output ready: /read_pose/*")
            return True
        except Exception as exc:
            self.available = False
            self.reason = str(exc)
            self.enabled = False
            self.logger.warning("ROS 2 input output unavailable: %s", exc)
            return False

    def publish(self, frame: dict) -> None:
        if not self.enabled or self.node is None:
            return
        message = self._PoseArray()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "pose_input_horizon"
        for side in ("left", "right"):
            sample = frame[side]
            pose = self._Pose()
            pose.position.x, pose.position.y, pose.position.z = sample["p"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = sample["q"]
            message.poses.append(pose)
        self._poses.publish(message)
        self._buttons.publish(self._UInt32(data=[frame["left"]["buttons"], frame["right"]["buttons"]]))
        self._tracked.publish(self._UInt8(data=[int(frame["left"]["valid"]), int(frame["right"]["valid"])]))

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
