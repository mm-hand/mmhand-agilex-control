from __future__ import annotations

import numpy as np

from config import IkConfig
from ik import NeroIK, _T_LINK7_TO_FLANGE
from ik_solver import ContinuityRuntimeState, NeroParams, fk, solve_pose_continuous_with_state


def _pose_error_norm(T_cur: np.ndarray, T_des: np.ndarray) -> float:
    dp = T_cur[:3, 3] - T_des[:3, 3]
    rot = 0.5 * (
        np.cross(T_cur[:3, 0], T_des[:3, 0])
        + np.cross(T_cur[:3, 1], T_des[:3, 1])
        + np.cross(T_cur[:3, 2], T_des[:3, 2])
    )
    return float(np.linalg.norm(np.concatenate([dp, rot])))


def test_solver_roundtrip() -> None:
    print("[test_solver_roundtrip] solve_pose_continuous_with_state")
    p = NeroParams.default()
    q_target = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    T_target = fk(q_target, p)
    state = ContinuityRuntimeState(q_prev=np.zeros(7))

    q_sol, report, next_state = solve_pose_continuous_with_state(T_target, state=state, p=p, n_psi=181)

    assert q_sol is not None
    assert report == {}
    assert next_state.theta0_prev is not None
    err = _pose_error_norm(fk(q_sol, p), T_target)
    print(f"  pose_err={err:.2e}")
    assert err < 1e-3


def test_neroik_end_to_end() -> None:
    print("[test_neroik_end_to_end] NeroIK.solve(flange_target)")
    ik = NeroIK(IkConfig())
    p = NeroParams.default()
    q_target = np.array([0.3, -0.2, 0.5, 0.8, -0.3, 0.1, 0.2])
    T_link7 = fk(q_target, p)
    T_flange = T_link7 @ _T_LINK7_TO_FLANGE

    q_sol = ik.solve(T_flange)

    assert q_sol is not None
    assert q_sol.shape == (7,)
    err = _pose_error_norm(fk(q_sol, p), T_link7)
    print(f"  pose_err={err:.2e}")
    assert err < 1e-3


def test_continuous_state_reuse() -> None:
    print("[test_continuous_state_reuse] repeated solves update state")
    p = NeroParams.default()
    state = ContinuityRuntimeState(q_prev=np.zeros(7))
    targets = [
        np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
        np.array([0.02, -0.02, 0.03, 1.02, -0.01, 0.01, 0.02]),
    ]

    for q_target in targets:
        q_sol, report, state = solve_pose_continuous_with_state(fk(q_target, p), state=state, p=p, n_psi=181)
        assert report == {}
        assert q_sol is not None
        assert state.q_prev.shape == (7,)


if __name__ == "__main__":
    test_solver_roundtrip()
    test_neroik_end_to_end()
    test_continuous_state_reuse()
    print("All IK tests passed.")
