from __future__ import annotations
from pathlib import Path
import numpy as np
from config import IkConfig
from ik_solver import ContinuityParams, ContinuityRuntimeState, NeroParams, solve_pose_continuous_with_state
from transforms import invert_transform, rotation_matrix_from_rpy_deg, transform_from_rotation_and_translation
_T_LINK7_TO_FLANGE = transform_from_rotation_and_translation(
    rotation_matrix_from_rpy_deg((-90.0, 0.0, -90.0)),
    (0.032, 0.0, -0.0235),
)
_T_FLANGE_TO_LINK7 = invert_transform(_T_LINK7_TO_FLANGE)
class NeroIK:
    def __init__(self, ik_config: IkConfig) -> None:
        import roboticstoolbox as rtb
        self.config = ik_config
        urdf_path = Path(ik_config.urdf_path)
        if not urdf_path.is_absolute():
            urdf_path = Path(__file__).resolve().parent / urdf_path
        links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(str(urdf_path), tld=str(urdf_path.parent))
        self.robot = rtb.Robot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath)
        self._p = NeroParams.default()
        self._n_psi = ik_config.n_psi
        self._continuity = ContinuityParams(
            local_theta0_window=ik_config.local_theta0_window,
            local_theta0_count=ik_config.local_theta0_count,
            w_vel=ik_config.w_vel,
            w_acc=ik_config.w_acc,
            w_pose=ik_config.w_pose,
            w_theta0=ik_config.w_theta0,
            w_limit=ik_config.w_limit,
            branch_switch_max_joint_delta_rad=ik_config.branch_switch_max_joint_delta_rad,
            branch_switch_min_improvement=ik_config.branch_switch_min_improvement,
            enable_initial_global_search=ik_config.enable_initial_global_search,
            enable_global_fallback=ik_config.enable_global_fallback,
        )
        self._state = ContinuityRuntimeState(q_prev=self.robot.q.copy())
    def reset_state(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=float).reshape(7)
        self.robot.q = q.copy()
        self._state = ContinuityRuntimeState(q_prev=q.copy())
    def solve(self, transform_arm_base_to_flange: np.ndarray) -> np.ndarray | None:
        q, _report, self._state = solve_pose_continuous_with_state(
            self._clamp_target_reach(transform_arm_base_to_flange @ _T_FLANGE_TO_LINK7),
            state=self._state,
            p=self._p,
            n_psi=self._n_psi,
            continuity=self._continuity,
        )
        return q
    def _clamp_target_reach(self, T07: np.ndarray) -> np.ndarray:
        p = self._p
        d1, d6, d8 = float(p.d_i[0]), float(p.d_i[6]), p.post_transform_d8
        z7 = T07[:3, 2]
        W = T07[:3, 3] - d8 * z7 - d6 * z7
        S = np.array([0.0, 0.0, d1])
        SW = W - S
        dist = float(np.linalg.norm(SW))
        if dist < 1e-10:
            return T07
        l_se, l_ew = abs(float(p.d_i[2])), abs(float(p.d_i[4]))
        target_dist = max(
            self.config.reach_min_ratio * abs(l_se - l_ew),
            min(self.config.reach_max_ratio * (l_se + l_ew), dist),
        )
        if target_dist == dist:
            return T07
        out = T07.copy()
        out[:3, 3] = S + SW * (target_dist / dist) + (d6 + d8) * z7
        return out
