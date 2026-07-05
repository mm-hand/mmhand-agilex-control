from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from config import MotionConfig
from ik_solver import NeroParams


def angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + np.pi) % (2.0 * np.pi) - np.pi


def max_abs_angle_diff(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.max(np.abs(angle_diff(a, b))))


def near_target(q: np.ndarray, target: np.ndarray) -> np.ndarray:
    return q - 2.0 * np.pi * np.round((q - target) / (2.0 * np.pi))


def joint_limits_with_margin(config: MotionConfig) -> tuple[np.ndarray, np.ndarray]:
    limits = np.asarray(NeroParams.default().joint_limits, dtype=float)
    margin = float(config.joint_limit_margin_rad)
    return limits[:, 0] + margin, limits[:, 1] - margin


def clip_to_joint_limits(q: np.ndarray, config: MotionConfig) -> np.ndarray:
    lo, hi = joint_limits_with_margin(config)
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

        target = near_target(np.asarray(q_target, dtype=float).reshape(7), self.q)
        target = clip_to_joint_limits(target, self.config)
        dq = angle_diff(target, self.q)

        vmax = _as_limit(self.config.max_velocity_rad_s, "max_velocity_rad_s")
        amax = _as_limit(self.config.max_acceleration_rad_s2, "max_acceleration_rad_s2")
        qd_des = np.clip(dq / dt, -vmax, vmax)
        dqd = np.clip(qd_des - self.qd, -amax * dt, amax * dt)
        qd_next = self.qd + dqd
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


def plan_joint_trajectory(config: MotionConfig, q_start: np.ndarray, q_target: np.ndarray) -> np.ndarray:
    import roboticstoolbox as rtb

    q0 = clip_to_joint_limits(np.asarray(q_start, dtype=float).reshape(7), config)
    q1 = clip_to_joint_limits(near_target(np.asarray(q_target, dtype=float).reshape(7), q0), config)
    dq = np.abs(angle_diff(q1, q0))
    vmax = _as_limit(config.max_velocity_rad_s, "max_velocity_rad_s")
    amax = _as_limit(config.max_acceleration_rad_s2, "max_acceleration_rad_s2")
    t_vel = float(np.max(dq / vmax))
    t_acc = float(np.max(np.sqrt(dq / amax)))
    duration = float(np.clip(max(t_vel, t_acc, config.min_trajectory_duration_s), config.min_trajectory_duration_s, config.max_trajectory_duration_s))
    steps = max(2, int(np.ceil(duration * config.command_hz)) + 1)
    t = np.linspace(0.0, duration, steps)
    traj = rtb.jtraj(q0, q1, t, qd0=np.zeros(7), qd1=np.zeros(7))
    return np.asarray(traj.q, dtype=float)
