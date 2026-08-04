from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np
from config import InputLimiterConfig, OneEuroConfig
from frames import ControllerPose
from transforms import normalize_quat_xyzw
class LowPass:
    def __init__(self) -> None:
        self.value: np.ndarray | None = None
    def apply(self, value: np.ndarray, alpha: float | np.ndarray) -> np.ndarray:
        if self.value is None:
            self.value = value.astype(float, copy=True)
        else:
            self.value = alpha * value + (1.0 - alpha) * self.value
        return self.value.copy()
class OneEuroFilter:
    def __init__(self, config: OneEuroConfig, size: int) -> None:
        self.config = config
        self.x_filter = LowPass()
        self.dx_filter = LowPass()
        self.last_t: float | None = None
        self.last_raw: np.ndarray | None = None
        self.size = size
    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)
    def reset(self) -> None:
        self.x_filter = LowPass()
        self.dx_filter = LowPass()
        self.last_t = None
        self.last_raw = None
    def apply(self, value, t: float) -> np.ndarray:
        x = np.asarray(value, dtype=float).reshape(self.size)
        if not self.config.enabled:
            self.last_t = t
            self.last_raw = x.copy()
            return x.copy()
        if self.last_t is None or t <= self.last_t:
            self.last_t = t
            self.last_raw = x.copy()
            return self.x_filter.apply(x, 1.0)
        dt = t - self.last_t
        dx = (x - self.last_raw) / dt if self.last_raw is not None else np.zeros_like(x)
        edx = self.dx_filter.apply(dx, self._alpha(self.config.d_cutoff, dt))
        cutoff = self.config.min_cutoff + self.config.beta * np.abs(edx)
        alpha = np.array([self._alpha(float(c), dt) for c in cutoff], dtype=float)
        self.last_t = t
        self.last_raw = x.copy()
        return self.x_filter.apply(x, alpha)
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
def _nearest_joint_angles(q: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return q - 2.0 * np.pi * np.round((q - reference) / (2.0 * np.pi))
@dataclass
class PoseInputLimiter:
    config: InputLimiterConfig
    def __post_init__(self) -> None:
        self.last_pose: ControllerPose | None = None
        self.last_t: float | None = None
    def apply(self, pose: ControllerPose | None, t: float) -> ControllerPose | None:
        if pose is None:
            return None
        if not self.config.enabled or self.last_pose is None or self.last_t is None or t <= self.last_t:
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
@dataclass
class PoseFilter:
    config: OneEuroConfig
    def __post_init__(self) -> None:
        self.position = OneEuroFilter(self.config, 3)
        self.quaternion = OneEuroFilter(self.config, 4)
        self.last_quat: np.ndarray | None = None
    def apply(self, pose: ControllerPose | None, t: float) -> ControllerPose | None:
        if pose is None:
            self.position.reset()
            self.quaternion.reset()
            self.last_quat = None
            return None
        xyz = self.position.apply(np.asarray(pose.xyz_m, dtype=float), t)
        quat = normalize_quat_xyzw(np.asarray(pose.quat_xyzw, dtype=float))
        if self.last_quat is not None and float(np.dot(quat, self.last_quat)) < 0.0:
            quat = -quat
        quat = normalize_quat_xyzw(self.quaternion.apply(quat, t))
        self.last_quat = quat.copy()
        return ControllerPose(tuple(float(v) for v in xyz), tuple(float(v) for v in quat), pose.received_at)
@dataclass
class JointTargetFilter:
    config: OneEuroConfig
    def __post_init__(self) -> None:
        self.filter = OneEuroFilter(self.config, 7)
        self.last_q: np.ndarray | None = None
    def reset(self) -> None:
        self.filter.reset()
        self.last_q = None
    def apply(self, q: np.ndarray | None, t: float) -> np.ndarray | None:
        if q is None:
            self.reset()
            return None
        target = np.asarray(q, dtype=float).reshape(7)
        if not np.all(np.isfinite(target)):
            raise ValueError("joint target contains NaN or Inf")
        if self.last_q is not None:
            target = _nearest_joint_angles(target, self.last_q)
        filtered = self.filter.apply(target, t)
        self.last_q = filtered.copy()
        return filtered
