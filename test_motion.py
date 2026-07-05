from __future__ import annotations

import numpy as np

from config import MotionConfig
from motion import ArmMotionPlanner, clip_to_joint_limits, plan_joint_trajectory


def _cfg() -> MotionConfig:
    return MotionConfig(
        command_hz=50.0,
        max_joint_step_rad=0.02,
        max_velocity_rad_s=(0.8,) * 7,
        max_acceleration_rad_s2=(2.0,) * 7,
        min_trajectory_duration_s=0.2,
        max_trajectory_duration_s=1.0,
    )


def test_online_servo_limits_first_step_acceleration():
    cfg = _cfg()
    planner = ArmMotionPlanner(cfg, np.zeros(7))
    q = planner.step(np.ones(7), 0.0)
    dt = 1.0 / cfg.command_hz
    assert np.max(np.abs(q)) <= 2.0 * dt * dt + 1e-12


def test_online_servo_limits_step_and_velocity():
    cfg = _cfg()
    planner = ArmMotionPlanner(cfg, np.zeros(7))
    last = planner.q.copy()
    for i in range(1, 80):
        q = planner.step(np.ones(7), i / cfg.command_hz)
        step = np.max(np.abs(q - last))
        assert step <= cfg.max_joint_step_rad + 1e-12
        assert np.max(np.abs(planner.qd)) <= max(cfg.max_velocity_rad_s) + 1e-12
        last = q.copy()


def test_none_target_holds_and_zeros_velocity():
    planner = ArmMotionPlanner(_cfg(), np.zeros(7))
    planner.step(np.ones(7), 0.0)
    held = planner.step(None, 0.02)
    assert np.allclose(held, planner.q)
    assert np.allclose(planner.qd, 0.0)


def test_joint_limit_clip_adds_margin():
    cfg = _cfg()
    q = np.ones(7) * 100.0
    clipped = clip_to_joint_limits(q, cfg)
    assert np.all(clipped < q)


def test_startup_jtraj_helper_has_endpoints():
    cfg = _cfg()
    q0 = np.zeros(7)
    q1 = np.ones(7) * 0.2
    traj = plan_joint_trajectory(cfg, q0, q1)
    assert traj.shape[1] == 7
    assert len(traj) > 2
    assert np.allclose(traj[0], q0)
    assert np.max(np.abs(traj[-1] - q1)) < 1e-9


if __name__ == "__main__":
    test_online_servo_limits_first_step_acceleration()
    test_online_servo_limits_step_and_velocity()
    test_none_target_holds_and_zeros_velocity()
    test_joint_limit_clip_adds_margin()
    test_startup_jtraj_helper_has_endpoints()
    print("All motion tests passed.")
