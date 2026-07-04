from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from config import MotionConfig


def _angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return (a - b + np.pi) % (2.0 * np.pi) - np.pi


def _max_abs_angle_diff(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.max(np.abs(_angle_diff(a, b))))


def _near_target(q: np.ndarray, target: np.ndarray) -> np.ndarray:
    return q - 2.0 * np.pi * np.round((q - target) / (2.0 * np.pi))


@dataclass
class ArmMotionPlanner:
    config: MotionConfig
    q: np.ndarray

    def __post_init__(self) -> None:
        self.q = np.asarray(self.q, dtype=float).reshape(7)
        self.qd = np.zeros(7, dtype=float)
        self.traj_q: np.ndarray | None = None
        self.traj_qd: np.ndarray | None = None
        self.traj_i = 0
        self.traj_goal: np.ndarray | None = None
        self.last_replan_at = -1e9

    def step(self, q_target: np.ndarray | None, now: float) -> np.ndarray:
        if q_target is None:
            self._clear_trajectory()
            self.qd[:] = 0.0
            return self.q.copy()

        q_target = _near_target(np.asarray(q_target, dtype=float).reshape(7), self.q)

        if self.traj_q is not None:
            self._advance_trajectory()
            if self._should_replan(q_target, now):
                self._plan(q_target, now)
            return self.q.copy()

        if _max_abs_angle_diff(q_target, self.q) > self.config.blend_trigger_rad:
            self._plan(q_target, now)
            self._advance_trajectory()
            return self.q.copy()

        dt = 1.0 / self.config.command_hz
        self.qd = _angle_diff(q_target, self.q) / dt
        self.q = q_target.copy()
        return self.q.copy()

    def _should_replan(self, q_target: np.ndarray, now: float) -> bool:
        if self.traj_goal is None:
            return True
        target_moved = _max_abs_angle_diff(q_target, self.traj_goal) > self.config.replan_trigger_rad
        can_replan = now - self.last_replan_at >= self.config.min_replan_interval_s
        return target_moved and can_replan

    def _advance_trajectory(self) -> None:
        if self.traj_q is None or self.traj_qd is None:
            return
        i = min(self.traj_i, len(self.traj_q) - 1)
        self.q = self.traj_q[i].copy()
        self.qd = self.traj_qd[i].copy()
        self.traj_i += 1
        if self.traj_i >= len(self.traj_q):
            self._clear_trajectory()
            self.qd[:] = 0.0

    def _plan(self, q_target: np.ndarray, now: float) -> None:
        import roboticstoolbox as rtb

        duration = self._duration(q_target)
        steps = max(2, int(np.ceil(duration * self.config.command_hz)) + 1)
        t = np.linspace(0.0, duration, steps)
        traj = rtb.jtraj(self.q, q_target, t, qd0=self.qd, qd1=np.zeros(7, dtype=float))
        duration = self._retimed_duration(duration, traj.qd, traj.qdd)
        if duration > t[-1] + 1e-9:
            steps = max(2, int(np.ceil(duration * self.config.command_hz)) + 1)
            t = np.linspace(0.0, duration, steps)
            traj = rtb.jtraj(self.q, q_target, t, qd0=self.qd, qd1=np.zeros(7, dtype=float))
        self.traj_q = np.asarray(traj.q, dtype=float)
        self.traj_qd = np.asarray(traj.qd, dtype=float)
        self.traj_i = 0
        self.traj_goal = q_target.copy()
        self.last_replan_at = now

    def _duration(self, q_target: np.ndarray) -> float:
        dq = np.abs(_angle_diff(q_target, self.q))
        vmax = np.asarray(self.config.max_velocity_rad_s, dtype=float)
        amax = np.asarray(self.config.max_acceleration_rad_s2, dtype=float)
        t_vel = float(np.max(dq / np.maximum(vmax, 1e-9)))
        t_acc = float(np.max(np.sqrt(dq / np.maximum(amax, 1e-9))))
        return float(np.clip(max(t_vel, t_acc, self.config.min_duration_s), self.config.min_duration_s, self.config.max_duration_s))

    def _retimed_duration(self, duration: float, qd: np.ndarray, qdd: np.ndarray) -> float:
        vmax = np.asarray(self.config.max_velocity_rad_s, dtype=float)
        amax = np.asarray(self.config.max_acceleration_rad_s2, dtype=float)
        v_scale = float(np.max(np.max(np.abs(qd), axis=0) / np.maximum(vmax, 1e-9)))
        a_scale = float(np.max(np.sqrt(np.max(np.abs(qdd), axis=0) / np.maximum(amax, 1e-9))))
        scale = max(1.0, v_scale, a_scale)
        return float(np.clip(duration * scale, self.config.min_duration_s, self.config.max_duration_s))

    def _clear_trajectory(self) -> None:
        self.traj_q = None
        self.traj_qd = None
        self.traj_i = 0
        self.traj_goal = None
