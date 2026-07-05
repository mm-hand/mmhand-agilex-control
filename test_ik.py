"""Verification tests for analytic arm-angle IK replacement.

Run: python3 test_ik.py
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from config import IkConfig
from ik import NeroIK, _T_FLANGE_TO_LINK7, _T_LINK7_TO_FLANGE
from ik_solver import (
    ContinuityParams,
    ContinuityRuntimeState,
    NeroParams,
    fk,
    joint_limit_cost,
    pose_error,
    solve_pose_continuous_with_state,
)


def _load_rtb_robot():
    import roboticstoolbox as rtb

    urdf_path = Path(__file__).resolve().parent / "nero_urdf" / "nero.urdf"
    links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
        str(urdf_path), tld=str(urdf_path.parent)
    )
    return rtb.Robot(
        links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath
    )


def _random_q(p: NeroParams, n: int, seed: int = 42) -> list[np.ndarray]:
    rng = np.random.default_rng(seed)
    qs = []
    while len(qs) < n:
        q = rng.uniform(p.joint_limits[:, 0], p.joint_limits[:, 1])
        if abs(q[3]) > 0.7:  # skip near-singular (fully extended) configs
            qs.append(q)
    return qs


def test_fk_consistency():
    """Verify ik_solver.fk(q) matches roboticstoolbox URDF FK to link7."""
    print("[test_fk_consistency] ik_solver.fk vs roboticstoolbox FK (end=link7)")
    robot = _load_rtb_robot()
    p = NeroParams.default()

    test_qs = [np.zeros(7), np.array([0.5, -0.3, 0.8, 1.0, -0.5, 0.2, 0.3])]
    test_qs += _random_q(p, 5, seed=42)

    max_pos = 0.0
    max_rot = 0.0
    for i, q in enumerate(test_qs):
        T_rtb = robot.fkine(q, end="link7").A
        T_ik = fk(q, p)
        pos_err = float(np.linalg.norm(T_rtb[:3, 3] - T_ik[:3, 3]))
        rot_err = float(np.linalg.norm(T_rtb[:3, :3] - T_ik[:3, :3]))
        max_pos = max(max_pos, pos_err)
        max_rot = max(max_rot, rot_err)
        print(f"  q[{i}]: pos_err={pos_err:.2e}, rot_err={rot_err:.2e}")

    print(f"  max pos_err={max_pos:.2e}, max rot_err={max_rot:.2e}")
    assert max_pos < 1e-4, f"FK position mismatch: {max_pos}"
    assert max_rot < 1e-4, f"FK rotation mismatch: {max_rot}"
    print("  PASSED\n")


def test_frame_conversion():
    """Verify T_link7 = T_flange @ _T_FLANGE_TO_LINK7 via roboticstoolbox FK."""
    print("[test_frame_conversion] flange->link7 transform")
    robot = _load_rtb_robot()
    p = NeroParams.default()

    test_qs = [np.zeros(7)] + _random_q(p, 5, seed=123)

    max_pos = 0.0
    max_rot = 0.0
    for i, q in enumerate(test_qs):
        T_link7 = robot.fkine(q, end="link7").A
        T_flange = robot.fkine(q, end="revo2_flange").A
        T_link7_via = T_flange @ _T_FLANGE_TO_LINK7
        pos_err = float(np.linalg.norm(T_link7[:3, 3] - T_link7_via[:3, 3]))
        rot_err = float(np.linalg.norm(T_link7[:3, :3] - T_link7_via[:3, :3]))
        max_pos = max(max_pos, pos_err)
        max_rot = max(max_rot, rot_err)
        print(f"  q[{i}]: pos_err={pos_err:.2e}, rot_err={rot_err:.2e}")

    print(f"  max pos_err={max_pos:.2e}, max rot_err={max_rot:.2e}")
    assert max_pos < 1e-10, f"Frame conversion position mismatch: {max_pos}"
    assert max_rot < 1e-10, f"Frame conversion rotation mismatch: {max_rot}"
    print("  PASSED\n")


def test_ik_roundtrip():
    """Verify IK round-trip: fk(ik(target)) ~= target."""
    print("[test_ik_roundtrip] fk(solve_pose_continuous(target)) ~= target")
    p = NeroParams.default()

    # Avoid q4≈0 (fully extended arm) — arm-angle parameterization singular there.
    test_qs = [np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])]
    test_qs += _random_q(p, 5, seed=456)

    max_err = 0.0
    for i, q_target in enumerate(test_qs):
        T_target = fk(q_target, p)
        state = ContinuityRuntimeState(q_prev=np.zeros(7))
        q_sol, report, _ = solve_pose_continuous_with_state(
            T_target, state=state, p=p, n_psi=181
        )
        assert q_sol is not None, f"IK failed for q[{i}]"
        T_sol = fk(q_sol, p)
        err = float(np.linalg.norm(pose_error(T_sol, T_target)))
        max_err = max(max_err, err)
        print(f"  q[{i}]: pose_err={err:.2e}, method={report.get('method', '?')}")

    print(f"  max pose_err={max_err:.2e}")
    assert max_err < 1e-3, f"IK round-trip error too large: {max_err}"
    print("  PASSED\n")


def test_neroik_end_to_end():
    """Verify NeroIK.solve() end-to-end with flange target."""
    print("[test_neroik_end_to_end] NeroIK.solve(flange_target)")
    ik_cfg = IkConfig()
    ik = NeroIK(ik_cfg)
    p = NeroParams.default()

    test_qs = [np.array([0.3, -0.2, 0.5, 0.8, -0.3, 0.1, 0.2])]
    test_qs += _random_q(p, 3, seed=999)
    max_err = 0.0
    for i, q_test in enumerate(test_qs):
        T_link7 = fk(q_test, p)
        T_flange = T_link7 @ _T_LINK7_TO_FLANGE
        q = ik.solve(T_flange)
        assert q is not None, f"NeroIK.solve() returned None for q[{i}]"
        assert q.shape == (7,), f"Wrong q shape: {q.shape}"
        T_result = fk(q, p)
        err = float(np.linalg.norm(pose_error(T_result, T_link7)))
        max_err = max(max_err, err)
        print(f"  q[{i}]: pose_err={err:.2e}")
        ik.robot.q = q

    print(f"  max pose_err={max_err:.2e}")
    assert max_err < 1e-3, f"End-to-end pose error too large: {max_err}"
    print("  PASSED\n")




def test_neroik_report_diagnostics():
    """NeroIK.solve_with_report should preserve solve() behavior and expose diagnostics."""
    print("[test_neroik_report_diagnostics] report fields")
    ik = NeroIK(IkConfig())
    p = NeroParams.default()
    q_test = np.array([0.2, -0.1, 0.4, 0.9, -0.2, 0.2, 0.1])
    T_flange = fk(q_test, p) @ _T_LINK7_TO_FLANGE
    q, report = ik.solve_with_report(T_flange)
    assert q is not None
    assert report["ok"] is True
    assert report["failure_reason"] is None
    assert report["solve_ms"] >= 0.0
    assert "target_clamp" in report
    assert "diagnostics" in report
    assert "target_reach" in report["diagnostics"]
    assert "branch_selected" in report
    print("  PASSED\n")


def test_target_clamp_report():
    """Targets outside the safe SEW reach range should be logged as clamped."""
    print("[test_target_clamp_report] far target clamp")
    ik = NeroIK(IkConfig())
    T_flange = np.eye(4)
    T_flange[:3, 3] = np.array([2.0, 0.0, 0.2])
    _q, report = ik.solve_with_report(T_flange)
    clamp = report["target_clamp"]
    assert clamp["clamped"] is True
    assert clamp["reason"] == "too_far"
    assert clamp["wrist_dist_after_m"] <= clamp["max_safe_m"] + 1e-12
    assert "diagnostics" in report
    print("  PASSED\n")


def test_joint_limit_cost():
    """Joint-limit soft cost should be low at center and high near limits."""
    print("[test_joint_limit_cost] center vs near limit")
    p = NeroParams.default()
    q_center = np.mean(p.joint_limits, axis=1)
    q_near = q_center.copy()
    q_near[0] = p.joint_limits[0, 1] - 0.02 * (p.joint_limits[0, 1] - p.joint_limits[0, 0])
    center_cost = joint_limit_cost(q_center, p)
    near_cost = joint_limit_cost(q_near, p)
    print(f"  center={center_cost:.3e} near={near_cost:.3e}")
    assert center_cost < 1e-9
    assert near_cost > center_cost + 10.0
    print("  PASSED\n")


def test_branch_lock_rejects_wrong_branch():
    """After acquisition, an impossible branch id must hold instead of switching."""
    print("[test_branch_lock_rejects_wrong_branch] strict analytic branch id")
    p = NeroParams.default()
    q0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    state0 = ContinuityRuntimeState(q_prev=np.zeros(7))
    q_first, report_first, state = solve_pose_continuous_with_state(
        fk(q0, p), state=state0, p=p, n_psi=181
    )
    assert q_first is not None
    assert state.branch_prev == report_first["branch_selected"]

    wrong_state = ContinuityRuntimeState(
        q_prev=state.q_prev,
        q_prev2=state.q_prev2,
        theta0_prev=state.theta0_prev,
        branch_prev=(9, 9, 9),
    )
    q_next, report_next, next_state = solve_pose_continuous_with_state(
        fk(q0, p), state=wrong_state, p=p, n_psi=181
    )
    assert q_next is None
    assert report_next["selected_by"] == "branch_lost"
    assert next_state is wrong_state
    print("  PASSED\n")

def test_global_search_only_for_initial_acquire():
    """Initial solve may scan globally; locked state must not globally reacquire."""
    print("[test_global_search_only_for_initial_acquire] initial global only")
    p = NeroParams.default()
    cont = ContinuityParams(
        local_theta0_window=0.0,
        local_theta0_count=11,
        enable_initial_global_search=True,
        enable_global_fallback=False,
    )

    q0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    state0 = ContinuityRuntimeState(q_prev=np.zeros(7))
    q_first, report_first, state = solve_pose_continuous_with_state(
        fk(q0, p), state=state0, p=p, n_psi=181, continuity=cont
    )
    assert q_first is not None
    assert report_first["method"] == "continuous_global_fallback"

    for q_far in _random_q(p, 80, seed=2026):
        q_next, report_next, _ = solve_pose_continuous_with_state(
            fk(q_far, p), state=state, p=p, n_psi=181, continuity=cont
        )
        if q_next is None:
            assert report_next["method"] == "continuous_local_theta0"
            assert report_next["failure_reason"] == "local_window_empty_global_disabled"
            assert "diagnostics" in report_next
            print("  PASSED\n")
            return

    raise AssertionError("Could not find a reachable target outside the locked local theta0 window")

def test_performance():
    """Measure single-solve time at 50 Hz target."""
    print("[test_performance] solve time benchmark")
    p = NeroParams.default()
    cfg = IkConfig()

    rng = np.random.default_rng(789)
    q = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    targets = []
    for _ in range(200):
        q = q + rng.uniform(-0.1, 0.1, 7)
        q = np.clip(q, p.joint_limits[:, 0], p.joint_limits[:, 1])
        targets.append(fk(q, p))

    cont = ContinuityParams(
        local_theta0_count=cfg.local_theta0_count,
        local_theta0_window=cfg.local_theta0_window,
    )
    state = ContinuityRuntimeState(q_prev=np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    _, _, state = solve_pose_continuous_with_state(targets[0], state=state, p=p, continuity=cont)
    times = []
    for T in targets[1:]:
        t0 = time.perf_counter()
        _, _, state = solve_pose_continuous_with_state(T, state=state, p=p, continuity=cont)
        times.append(time.perf_counter() - t0)

    avg_ms = np.mean(times) * 1000
    p95_ms = np.percentile(times, 95) * 1000
    max_ms = np.max(times) * 1000
    ok = "OK" if max_ms < 20.0 else "TOO SLOW"
    print(f"  avg={avg_ms:.2f}ms  p95={p95_ms:.2f}ms  max={max_ms:.2f}ms  [{ok}]")
    print("  PASSED\n")


if __name__ == "__main__":
    test_fk_consistency()
    test_frame_conversion()
    test_ik_roundtrip()
    test_neroik_end_to_end()
    test_neroik_report_diagnostics()
    test_target_clamp_report()
    test_joint_limit_cost()
    test_branch_lock_rejects_wrong_branch()
    test_global_search_only_for_initial_acquire()
    test_performance()
    print("All tests passed.")
