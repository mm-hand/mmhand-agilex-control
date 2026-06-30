from __future__ import annotations

import math
import re
import tempfile
from pathlib import Path

import numpy as np
from spatialmath import SE3

from .config import FrameTransformConfig, IkConfig
from .transforms import rotation_matrix_from_rpy_deg, transform_from_rotation_and_translation

_PACKAGE_PREFIX = "package://agx_arm_description/agx_arm_urdf/nero/"


def _resolve_urdf(urdf_path: str) -> str:
    """Read URDF, rewrite package:// mesh paths to absolute paths, return temp path."""
    urdf = Path(urdf_path).expanduser().resolve()
    mesh_root = urdf.parent.parent / "meshes"  # nero/meshes
    text = urdf.read_text(encoding="utf-8")
    text = text.replace(_PACKAGE_PREFIX, str(mesh_root) + "/")
    tmp = Path(tempfile.gettempdir()) / "nero_resolved.urdf"
    tmp.write_text(text, encoding="utf-8")
    return str(tmp)


def _link7_from_flange(config: FrameTransformConfig) -> np.ndarray:
    return transform_from_rotation_and_translation(
        rotation_matrix_from_rpy_deg(config.rotation_link7_from_flange_rpy_deg),
        config.translation_link7_from_flange_m,
    )


class NeroIK:
    """roboticstoolbox nero arm with flange injected as robot.tool."""

    def __init__(self, ik_config: IkConfig, frame_config: FrameTransformConfig) -> None:
        import roboticstoolbox as rtb

        self.config = ik_config
        self.robot = rtb.Robot.URDF(_resolve_urdf(ik_config.urdf_path))
        # Inject flange as the tool frame so IK targets the flange directly.
        T_link7_from_flange = _link7_from_flange(frame_config)
        R = T_link7_from_flange[:3, :3]
        t = T_link7_from_flange[:3, 3]
        self.robot.tool = SE3.Rt(R, t)
        self.q_prev = self.robot.q.copy()

    def solve(self, transform_arm_base_from_flange: np.ndarray) -> np.ndarray:
        R = transform_arm_base_from_flange[:3, :3]
        t = transform_arm_base_from_flange[:3, 3]
        # ik_LM returns (q, iterations, searches, reason, residual).
        # Always return the best-effort q (closest reachable pose), even if not
        # fully converged, so the arm tracks the target as closely as possible.
        sol = self.robot.ik_LM(
            SE3.Rt(R, t),
            q0=self.q_prev,
            tol=self.config.tol,
            ilimit=self.config.max_iter,
        )
        q = np.asarray(sol[0]).reshape(-1)
        self.q_prev = q
        return q
