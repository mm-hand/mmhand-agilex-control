from __future__ import annotations

from pathlib import Path

import numpy as np
from spatialmath import SE3

from config import IkConfig


class NeroIK:
    """Nero arm IK with revo2_flange as the end-effector link."""

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
        self.q_prev = self.robot.q.copy()

    def solve(self, transform_arm_base_to_flange: np.ndarray) -> np.ndarray | None:
        rotation = transform_arm_base_to_flange[:3, :3]
        translation = transform_arm_base_to_flange[:3, 3]
        sol = self.robot.ik_LM(
            SE3.Rt(rotation, translation),
            q0=self.q_prev,
            tol=self.config.tol,
            ilimit=self.config.max_iter,
            slimit=self.config.slimit,
            k=self.config.k,
            method=self.config.method,
            end="revo2_flange",
        )
        if int(sol[1]) <= 0:
            return None
        q = np.asarray(sol[0]).reshape(-1)
        self.q_prev = q
        return q
