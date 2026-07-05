from __future__ import annotations

from pathlib import Path

import numpy as np

from config import IkConfig
from ik_solver import (
    ContinuityParams,
    ContinuityRuntimeState,
    NeroParams,
    solve_pose_continuous_with_state,
)
from transforms import (
    invert_transform,
    rotation_matrix_from_rpy_rad,
    transform_from_rotation_and_translation,
)

# URDF revo2_flange_joint origin (parent=link7, child=revo2_flange):
#   xyz="0.032 0 -0.0235", rpy="-1.5708 0 -1.5708" (radians)
# Builds ^link7 T_flange, then inverts to ^flange T_link7 so that
# incoming flange targets can be converted to link7 targets for the IK.
_R_LINK7_TO_FLANGE = rotation_matrix_from_rpy_rad(-1.5708, 0.0, -1.5708)
_T_LINK7_TO_FLANGE = transform_from_rotation_and_translation(
    _R_LINK7_TO_FLANGE, (0.032, 0.0, -0.0235)
)
_T_FLANGE_TO_LINK7 = invert_transform(_T_LINK7_TO_FLANGE)


class NeroIK:
    """Nero arm IK with revo2_flange as the end-effector link.

    Uses analytic arm-angle parameterized IK (ik_solver) for solving and
    roboticstoolbox Robot for Swift visualization.
    """

    def __init__(self, ik_config: IkConfig) -> None:
        import roboticstoolbox as rtb

        self.config = ik_config
        urdf_path = Path(ik_config.urdf_path)
        if not urdf_path.is_absolute():
            urdf_path = Path(__file__).resolve().parent / urdf_path
        links, name, urdf_string, urdf_filepath = rtb.Robot.URDF_read(
            str(urdf_path), tld=str(urdf_path.parent)
        )
        self.robot = rtb.Robot(
            links,
            name=name,
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

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
            enable_initial_global_search=ik_config.enable_initial_global_search,
            enable_global_fallback=ik_config.enable_global_fallback,
        )
        self._state = ContinuityRuntimeState(q_prev=self.robot.q.copy())

    def solve(self, transform_arm_base_to_flange: np.ndarray) -> np.ndarray | None:
        T07 = transform_arm_base_to_flange @ _T_FLANGE_TO_LINK7
        T07 = self._clamp_target_reach(T07)
        q, _report, self._state = solve_pose_continuous_with_state(
            T07,
            state=self._state,
            p=self._p,
            n_psi=self._n_psi,
            continuity=self._continuity,
        )
        return q

    def _clamp_target_reach(self, T07: np.ndarray) -> np.ndarray:
        """钳制目标位姿到安全可达范围，避免肘部奇异。

        Nero 上臂长 l_se = 0.31m，前臂长 l_ew = 0.27m。
        - 最大臂展 = l_se + l_ew = 0.58m：当 |SW| 接近此值时，
          SEW 三角形退化 (q4→0，肘部完全伸直)，臂角参数化 IK 无解。
        - 最小折叠 = |l_se - l_ew| = 0.04m：当 |SW| 接近此值时，
          三角形同样退化（肘部完全折叠）。

        本方法将腕心 W 沿肩→腕方向回缩/外推到安全范围，
        保持末端姿态(旋转)不变，仅修正平移分量。
        安全边界取 95%/105%，留 5% 余量确保 SEW 三角形非退化。
        """
        p = self._p
        # 从 T07 计算 S, W（与 ik_solver._compute_swe_from_target 相同的几何）
        d1 = float(p.d_i[0])     # 肩高度: 0.138
        d6 = float(p.d_i[6])     # 腕到 link7: 0.0235
        d8 = p.post_transform_d8  # post-transform: 0.0

        z7 = T07[:3, 2]                    # link7 z 轴 = 旋转矩阵第三列
        O7 = T07[:3, 3] - d8 * z7          # link7 原点（去除 post-transform）
        W = O7 - d6 * z7                   # 腕心 = link7 原点沿 z7 回退 d6
        S = np.array([0.0, 0.0, d1])       # 肩心 = base + d1·z

        SW = W - S
        dist = float(np.linalg.norm(SW))
        l_se = abs(float(p.d_i[2]))        # 0.31
        l_ew = abs(float(p.d_i[4]))        # 0.27
        max_safe = 0.95 * (l_se + l_ew)    # 0.551m
        min_safe = 1.05 * abs(l_se - l_ew) # 0.042m

        if dist < 1e-10 or min_safe <= dist <= max_safe:
            return T07  # 在安全范围内或退化情况，不修正

        # 钳制 |SW| 到安全范围
        target_dist = max(min_safe, min(max_safe, dist))
        scale = target_dist / dist
        W_clamped = S + SW * scale

        # 从修正后的 W 反算 T07 平移（保持旋转不变）
        O7_clamped = W_clamped + d6 * z7
        T07_clamped = T07.copy()
        T07_clamped[:3, 3] = O7_clamped + d8 * z7
        return T07_clamped
