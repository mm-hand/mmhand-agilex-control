from __future__ import annotations

import numpy as np

from config import ControlConfig, IkConfig, MotionConfig
from ik import NeroIK, _T_LINK7_TO_FLANGE
from ik_solver import ContinuityRuntimeState, NeroParams, fk, solve_pose_continuous_with_state
from motion import ArmMotionPlanner, clip_to_joint_limits, joint_trajectory, plan_joint_trajectory
from ui.scene import UrdfModel, link_frames


def _pose_error_norm(T_cur: np.ndarray, T_des: np.ndarray) -> float:
    dp = T_cur[:3, 3] - T_des[:3, 3]
    rot = 0.5 * (
        np.cross(T_cur[:3, 0], T_des[:3, 0])
        + np.cross(T_cur[:3, 1], T_des[:3, 1])
        + np.cross(T_cur[:3, 2], T_des[:3, 2])
    )
    return float(np.linalg.norm(np.concatenate([dp, rot])))


def _cfg() -> MotionConfig:
    return MotionConfig(
        command_hz=50.0,
        max_joint_step_rad=0.02,
        max_velocity_rad_s=(0.8,) * 7,
        max_acceleration_rad_s2=(2.0,) * 7,
        max_trajectory_duration_s=1.0,
    )


def test_solver_roundtrip():
    p = NeroParams.default()
    q_target = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    T_target = fk(q_target, p)
    state = ContinuityRuntimeState(q_prev=np.zeros(7))
    q_sol, report, next_state = solve_pose_continuous_with_state(T_target, state=state, p=p, n_psi=181)
    assert q_sol is not None
    assert report == {}
    assert next_state.theta0_prev is not None
    assert _pose_error_norm(fk(q_sol, p), T_target) < 1e-3


def test_neroik_end_to_end():
    solver = NeroIK(IkConfig())
    p = NeroParams.default()
    q_target = np.array([0.3, -0.2, 0.5, 0.8, -0.3, 0.1, 0.2])
    T_link7 = fk(q_target, p)
    q_sol = solver.solve(T_link7 @ _T_LINK7_TO_FLANGE)
    assert q_sol is not None
    assert q_sol.shape == (7,)
    assert _pose_error_norm(fk(q_sol, p), T_link7) < 1e-3


def test_continuous_state_reuse():
    p = NeroParams.default()
    state = ContinuityRuntimeState(q_prev=np.zeros(7))
    for q_target in (
        np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
        np.array([0.02, -0.02, 0.03, 1.02, -0.01, 0.01, 0.02]),
    ):
        q_sol, report, state = solve_pose_continuous_with_state(fk(q_target, p), state=state, p=p, n_psi=181)
        assert report == {}
        assert q_sol is not None
        assert state.q_prev.shape == (7,)


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
        assert np.max(np.abs(q - last)) <= cfg.max_joint_step_rad + 1e-12
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
    assert np.all(clip_to_joint_limits(q, cfg) < q)


def test_startup_jtraj_helper_has_endpoints():
    cfg = _cfg()
    q0 = np.zeros(7)
    q1 = np.ones(7) * 0.2
    traj = plan_joint_trajectory(cfg, q0, q1)
    assert traj.shape[1] == 7
    assert len(traj) > 2
    assert np.allclose(traj[0], q0)
    assert np.max(np.abs(traj[-1] - q1)) < 1e-9


def test_urdf_chain_matches_analytic_kinematics():
    model = UrdfModel.load()
    assert model.base_link == "base_link"
    assert model.revolute_joints == tuple(f"joint{index}" for index in range(1, 8))
    for q in (
        np.zeros(7),
        np.linspace(-0.5, 0.5, 7),
        np.array([0.3, -0.2, 0.5, 0.8, -0.3, 0.1, 0.2]),
    ):
        frames = link_frames(model, q)
        assert np.allclose(frames["link7"], fk(q), atol=1e-4)
        assert np.allclose(frames["gripper_flange"], fk(q) @ _T_LINK7_TO_FLANGE, atol=1e-4)


def test_urdf_visual_meshes_exist():
    model = UrdfModel.load()
    for link in model.links.values():
        if link.mesh is not None:
            assert link.mesh.is_file()


def test_joint_trajectory_reaches_far_pose():
    motion = MotionConfig()
    target = np.asarray(ControlConfig().startup.right_pose_rad, dtype=float)
    q_start = np.array([-1.29, -0.36, -1.43, 0.07, 0.31, -0.59, -1.33])
    points = joint_trajectory(motion, q_start, target)
    assert np.allclose(points[-1], clip_to_joint_limits(target, motion), atol=1e-6)


def test_joint_trajectory_does_not_wrap_across_joint_limits():
    motion = MotionConfig()
    target = np.asarray(ControlConfig().startup.right_pose_rad, dtype=float)
    q_start = clip_to_joint_limits(target, motion)
    q_start[4] = 2.70
    points = joint_trajectory(motion, q_start, target)
    clipped = clip_to_joint_limits(target, motion)
    assert points[-1][4] < 0.0
    assert abs(points[-1][4] - clipped[4]) < 1e-6
