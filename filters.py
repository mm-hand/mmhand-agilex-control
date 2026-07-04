from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from config import OneEuroConfig
from frames import ControllerPose
from transforms import normalize_quat_xyzw


class LowPass:
    def __init__(self) -> None:
        self.value: np.ndarray | None = None

    def apply(self, value: np.ndarray, alpha: float) -> np.ndarray:
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


@dataclass
class PoseFilter:
    config: OneEuroConfig

    def __post_init__(self) -> None:
        self.position = OneEuroFilter(self.config, 3)
        self.quaternion = OneEuroFilter(self.config, 4)
        self.last_quat: np.ndarray | None = None

    def apply(self, pose: ControllerPose | None, t: float) -> ControllerPose | None:
        if pose is None:
            return None
        xyz = self.position.apply(np.asarray(pose.xyz_m, dtype=float), t)
        quat = normalize_quat_xyzw(np.asarray(pose.quat_xyzw, dtype=float))
        if self.last_quat is not None and float(np.dot(quat, self.last_quat)) < 0.0:
            quat = -quat
        quat = normalize_quat_xyzw(self.quaternion.apply(quat, t))
        self.last_quat = quat.copy()
        return ControllerPose(tuple(float(v) for v in xyz), tuple(float(v) for v in quat), pose.received_at)
