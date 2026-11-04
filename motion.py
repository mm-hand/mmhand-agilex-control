from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from config import MotionConfig
from ik_solver import NeroParams
def joint_limits() -> tuple[np.ndarray, np.ndarray]:
    limits = np.asarray(NeroParams.default().joint_limits, dtype=float)
    return limits[:, 0].copy(), limits[:, 1].copy()
def joint_limits_with_margin(config: MotionConfig) -> tuple[np.ndarray, np.ndarray]:
    lower, upper = joint_limits()
    margin = float(config.joint_limit_margin_rad)
    return lower + margin, upper - margin
def clip_to_joint_limits(q: np.ndarray, config: MotionConfig) -> np.ndarray:
    lo, hi = joint_limits_with_margin(config)
    return np.clip(np.asarray(q, dtype=float).reshape(7), lo, hi)
def bounded_target(q_target: np.ndarray, q_reference: np.ndarray, config: MotionConfig) -> np.ndarray:
    """Nearest equivalent of q_target that stays inside the joint limits."""
    lo, hi = joint_limits_with_margin(config)
    target = np.asarray(q_target, dtype=float).reshape(7)
    reference = np.asarray(q_reference, dtype=float).reshape(7)
    result = np.empty(7, dtype=float)
    for index in range(7):
        candidates = [
            target[index] + 2.0 * np.pi * shift
            for shift in (-1, 0, 1)
            if lo[index] <= target[index] + 2.0 * np.pi * shift <= hi[index]
        ]
        if candidates:
            result[index] = min(candidates, key=lambda value: abs(value - reference[index]))
        else:
            result[index] = float(np.clip(target[index], lo[index], hi[index]))
    return result
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
        target = bounded_target(q_target, self.q, self.config)
        dq = target - self.q
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
    start = clip_to_joint_limits(q_start, config)
    planner = ArmMotionPlanner(config, start)
    target = bounded_target(q_target, planner.q, config)
    dt = 1.0 / config.command_hz
    max_steps = max(1, int(np.ceil(config.max_trajectory_duration_s / dt)))
    points: list[np.ndarray] = []
    for step in range(max_steps):
        points.append(planner.step(target, (step + 1) * dt))
        if np.max(np.abs(target - planner.q)) < 1e-6:
            return points
    raise RuntimeError(
        "trajectory did not reach its target: "
        f"start={np.round(start, 3).tolist()} target={np.round(target, 3).tolist()}"
    )
def plan_joint_trajectory(config: MotionConfig, q_start: np.ndarray, q_target: np.ndarray) -> np.ndarray:
    q0 = clip_to_joint_limits(q_start, config)
    return np.vstack([q0, *joint_trajectory(config, q0, q_target)])
