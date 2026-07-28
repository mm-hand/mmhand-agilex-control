from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from config import MotionConfig
from ik_solver import NeroParams
def angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + np.pi) % (2.0 * np.pi) - np.pi
def near_target(q: np.ndarray, target: np.ndarray) -> np.ndarray:
    return q - 2.0 * np.pi * np.round((q - target) / (2.0 * np.pi))
def clip_to_joint_limits(q: np.ndarray, config: MotionConfig) -> np.ndarray:
    limits = np.asarray(NeroParams.default().joint_limits, dtype=float)
    margin = float(config.joint_limit_margin_rad)
    lo, hi = limits[:, 0] + margin, limits[:, 1] - margin
    return np.clip(np.asarray(q, dtype=float).reshape(7), lo, hi)
def _as_limit(values: tuple[float, ...], name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=float).reshape(7)
    if np.any(arr <= 0.0) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain 7 positive finite values")
    return arr
@dataclass
class ArmMotionPlanner:
    config: MotionConfig
    q: np.ndarray
    def __post_init__(self) -> None:
        self.q = clip_to_joint_limits(np.asarray(self.q, dtype=float).reshape(7), self.config)
        self.qd = np.zeros(7, dtype=float)
        self.last_t: float | None = None
    def reset(self, q: np.ndarray) -> None:
        self.q = clip_to_joint_limits(np.asarray(q, dtype=float).reshape(7), self.config)
        self.qd[:] = 0.0
        self.last_t = None
    def step(self, q_target: np.ndarray | None, now: float) -> np.ndarray:
        if q_target is None:
            self.qd[:] = 0.0
            self.last_t = now
            return self.q.copy()
        if self.last_t is None or now <= self.last_t:
            dt = 1.0 / self.config.command_hz
        else:
            dt = min(max(now - self.last_t, 1e-4), 2.0 / self.config.command_hz)
        self.last_t = now
        target = clip_to_joint_limits(near_target(np.asarray(q_target, dtype=float).reshape(7), self.q), self.config)
        dq = angle_diff(target, self.q)
        vmax = _as_limit(self.config.max_velocity_rad_s, "max_velocity_rad_s")
        amax = _as_limit(self.config.max_acceleration_rad_s2, "max_acceleration_rad_s2")
        qd_next = self.qd + np.clip(np.clip(dq / dt, -vmax, vmax) - self.qd, -amax * dt, amax * dt)
        step = qd_next * dt
        max_step = float(self.config.max_joint_step_rad)
        if max_step > 0.0:
            step = np.clip(step, -max_step, max_step)
            qd_next = step / dt
        overshoot = np.abs(step) > np.abs(dq)
        step[overshoot] = dq[overshoot]
        qd_next[overshoot] = step[overshoot] / dt
        self.q = clip_to_joint_limits(self.q + step, self.config)
        self.qd = qd_next.copy()
        return self.q.copy()


def joint_trajectory(config: MotionConfig, q_start: np.ndarray, q_target: np.ndarray) -> list[np.ndarray]:
    planner = ArmMotionPlanner(config, q_start)
    target = clip_to_joint_limits(near_target(np.asarray(q_target, dtype=float).reshape(7), planner.q), config)
    vmax = _as_limit(config.max_velocity_rad_s, "max_velocity_rad_s")
    amax = _as_limit(config.max_acceleration_rad_s2, "max_acceleration_rad_s2")
    duration = np.max(np.abs(angle_diff(target, planner.q)) / vmax + 2.0 * vmax / amax) + 1.0
    dt = 1.0 / config.command_hz
    points: list[np.ndarray] = []
    for step in range(int(np.ceil(duration / dt))):
        points.append(planner.step(target, (step + 1) * dt))
        if np.max(np.abs(angle_diff(target, planner.q))) < 1e-6:
            return points
    raise RuntimeError("startup trajectory did not reach its target")
