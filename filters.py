from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np
from config import InputLimiterConfig
from frames import ControllerPose
from transforms import normalize_quat_xyzw
def _quat_slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = normalize_quat_xyzw(q0)
    q1 = normalize_quat_xyzw(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quat_xyzw((1.0 - alpha) * q0 + alpha * q1)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    a = math.sin((1.0 - alpha) * theta) / sin_theta
    b = math.sin(alpha * theta) / sin_theta
    return normalize_quat_xyzw(a * q0 + b * q1)
def _quat_angle(q0: np.ndarray, q1: np.ndarray) -> float:
    q0 = normalize_quat_xyzw(q0)
    q1 = normalize_quat_xyzw(q1)
    dot = abs(float(np.dot(q0, q1)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))
@dataclass
class PoseInputLimiter:
    config: InputLimiterConfig
    def __post_init__(self) -> None:
        self.last_pose: ControllerPose | None = None
        self.last_t: float | None = None
    def apply(self, pose: ControllerPose | None, t: float) -> ControllerPose | None:
        if pose is None:
            return None
        if self.last_pose is None or self.last_t is None or t <= self.last_t:
            self.last_pose = pose
            self.last_t = t
            return pose
        dt = t - self.last_t
        xyz_prev = np.asarray(self.last_pose.xyz_m, dtype=float)
        xyz_target = np.asarray(pose.xyz_m, dtype=float)
        dxyz = xyz_target - xyz_prev
        max_pos_step = min(self.config.max_position_step_m, self.config.max_linear_speed_m_s * dt)
        dist = float(np.linalg.norm(dxyz))
        if dist > max_pos_step > 0.0:
            xyz_target = xyz_prev + dxyz * (max_pos_step / dist)
        quat_prev = normalize_quat_xyzw(np.asarray(self.last_pose.quat_xyzw, dtype=float))
        quat_target = normalize_quat_xyzw(np.asarray(pose.quat_xyzw, dtype=float))
        angle = _quat_angle(quat_prev, quat_target)
        max_rot_step = min(self.config.max_rotation_step_rad, self.config.max_angular_speed_rad_s * dt)
        if angle > max_rot_step > 0.0:
            quat_target = _quat_slerp(quat_prev, quat_target, max_rot_step / angle)
        limited = ControllerPose(
            tuple(float(v) for v in xyz_target),
            tuple(float(v) for v in quat_target),
            pose.received_at,
        )
        self.last_pose = limited
        self.last_t = t
        return limited
