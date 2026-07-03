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
            hysteresis_margin=ik_config.hysteresis_margin,
            enable_global_fallback=ik_config.enable_global_fallback,
            enable_qp=ik_config.enable_qp,
            w_qp_joint_inc=ik_config.w_qp_joint_inc,
            w_qp_pose_err=ik_config.w_qp_pose_err,
        )
        self._state = ContinuityRuntimeState(q_prev=self.robot.q.copy())

    def solve(self, transform_arm_base_to_flange: np.ndarray) -> np.ndarray | None:
        T07 = transform_arm_base_to_flange @ _T_FLANGE_TO_LINK7
        q, _report, self._state = solve_pose_continuous_with_state(
            T07,
            state=self._state,
            p=self._p,
            n_psi=self._n_psi,
            continuity=self._continuity,
        )
        return q
