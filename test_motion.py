from __future__ import annotations

import numpy as np

from config import MotionConfig
from motion import ArmMotionPlanner
from control_loop import direct_joint_command


def _cfg() -> MotionConfig:
    return MotionConfig(
        command_hz=50.0,
        blend_trigger_rad=0.05,
        replan_trigger_rad=0.08,
        min_replan_interval_s=0.25,
        min_duration_s=0.2,
        max_duration_s=1.0,
        max_velocity_rad_s=(0.8,) * 7,
        max_acceleration_rad_s2=(2.0,) * 7,
    )


def test_initial_large_move_builds_trajectory():
    planner = ArmMotionPlanner(_cfg(), np.zeros(7))
    q = planner.step(np.ones(7) * 0.5, 0.0)
    assert planner.traj_q is not None
    assert len(planner.traj_q) > 2
    assert not np.allclose(q, np.ones(7) * 0.5)


def test_small_target_change_does_not_replan():
    planner = ArmMotionPlanner(_cfg(), np.zeros(7))
    planner.step(np.ones(7) * 0.5, 0.0)
    first_goal = planner.traj_goal.copy()
    first_plan = planner.traj_q
    planner.step(first_goal + 0.01, 0.30)
    assert planner.traj_q is first_plan


def test_large_target_change_replans_after_interval():
    planner = ArmMotionPlanner(_cfg(), np.zeros(7))
    planner.step(np.ones(7) * 0.5, 0.0)
    first_plan = planner.traj_q
    planner.step(np.ones(7) * -0.5, 0.30)
    assert planner.traj_q is not None
    assert planner.traj_q is not first_plan


def test_none_target_holds_and_clears_trajectory():
    planner = ArmMotionPlanner(_cfg(), np.zeros(7))
    planner.step(np.ones(7) * 0.5, 0.0)
    held = planner.step(None, 0.02)
    assert planner.traj_q is None
    assert np.allclose(held, planner.q)


def test_direct_joint_command_updates_or_holds():
    last = np.ones(7)
    target = np.ones(7) * 2.0
    assert np.allclose(direct_joint_command(target, last), target)
    assert np.allclose(direct_joint_command(None, last), last)


if __name__ == "__main__":
    test_initial_large_move_builds_trajectory()
    test_small_target_change_does_not_replan()
    test_large_target_change_replans_after_interval()
    test_none_target_holds_and_clears_trajectory()
    test_direct_joint_command_updates_or_holds()
    print("All motion tests passed.")
