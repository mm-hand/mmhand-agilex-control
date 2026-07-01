from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

import numpy as np
from spatialmath import SE3

from .config import IkConfig

_XACRO_PKG = "/tmp/xacro_extract/opt/ros/jazzy/lib/python3.12/site-packages"
_AGX_INSTALL = "/tmp/agx_install"
_MESH_PREFIX = "package://agx_arm_description/agx_arm_urdf/nero/"
_MESH_ROOT = str(Path(__file__).resolve().parent / "nero")


def _resolve_xacro(xacro_path: str) -> str:
    """Expand xacro to URDF, rewrite package:// mesh paths, return temp file path."""
    if _XACRO_PKG not in sys.path:
        sys.path.insert(0, _XACRO_PKG)
    os.environ["AMENT_PREFIX_PATH"] = _AGX_INSTALL
    import xacro

    doc = xacro.process_file(str(Path(xacro_path).resolve()))
    xml = doc.toxml().replace(_MESH_PREFIX, _MESH_ROOT + "/")
    tmp = Path(tempfile.gettempdir()) / "nero_xacro.urdf"
    tmp.write_text(xml, encoding="utf-8")
    return str(tmp)


class NeroIK:
    """roboticstoolbox nero arm loaded from xacro. IK targets revo2_flange directly."""

    def __init__(self, ik_config: IkConfig) -> None:
        import roboticstoolbox as rtb

        self.config = ik_config
        self.robot = rtb.Robot.URDF(_resolve_xacro(ik_config.xacro_path))
        self.q_prev = self.robot.q.copy()

    def solve(self, transform_arm_base_from_flange: np.ndarray) -> np.ndarray | None:
        R = transform_arm_base_from_flange[:3, :3]
        t = transform_arm_base_from_flange[:3, 3]
        # ik_LM returns (q, iterations, searches, reason, residual).
        # iterations > 0 means converged. On failure return None so the caller
        # holds the previous solvable q and reports IK unsolvable.
        sol = self.robot.ik_LM(
            SE3.Rt(R, t),
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
