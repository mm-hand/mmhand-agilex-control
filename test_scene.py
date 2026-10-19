from __future__ import annotations

import numpy as np

from ik import _T_LINK7_TO_FLANGE
from ik_solver import fk
from ui.scene import joint_frames


def test_joint_frames_match_analytic_forward_kinematics():
    for q in (
        np.zeros(7),
        np.linspace(-0.5, 0.5, 7),
        np.array([0.3, -0.2, 0.5, 0.8, -0.3, 0.1, 0.2]),
    ):
        frames = joint_frames(q)
        assert len(frames) == 8
        assert np.allclose(frames[-1], fk(q), atol=1e-9)
