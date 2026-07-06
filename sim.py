from __future__ import annotations

import logging
import numpy as np
from spatialmath import SE3
from spatialgeometry import Axes

from config import SimConfig
from frames import ArmBases, CoordinateFrames

# Swift world is Z-up; input horizon is Y-up.
# Swift_X = hX, Swift_Y = -hZ, Swift_Z = hY.
_SWIFT_FROM_HORIZON = np.eye(4)
_SWIFT_FROM_HORIZON[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])


def swift_se3(transform_horizon: np.ndarray, z_offset_m: float = 0.0) -> SE3:
    transform = _SWIFT_FROM_HORIZON @ transform_horizon
    transform[:3, 3] += np.array([0.0, 0.0, z_offset_m])
    return SE3.Rt(transform[:3, :3], transform[:3, 3])


class SwiftSimBackend:
    def __init__(self, left_robot, right_robot, bases: ArmBases, sim_config: SimConfig, logger: logging.Logger) -> None:
        from swift import Swift
        from swift.SwiftElement import Label

        self.logger = logger
        self.z_offset_m = sim_config.visual_z_offset_m
        self.left_robot = left_robot
        self.right_robot = right_robot
        self.left_robot.base = swift_se3(bases.left, self.z_offset_m)
        self.right_robot.base = swift_se3(bases.right, self.z_offset_m)

        self.left_input, self.right_input, self.left_flange, self.right_flange, self.left_base, self.right_base = (
            Axes(0.05) for _ in range(6)
        )
        self.horizon_frame = Axes(sim_config.horizon_frame_axis_length_m) if sim_config.show_horizon_frame else None
        self.left_base.T = swift_se3(bases.left, self.z_offset_m).A
        self.right_base.T = swift_se3(bases.right, self.z_offset_m).A
        if self.horizon_frame is not None:
            self.horizon_frame.T = swift_se3(np.eye(4), self.z_offset_m).A
        self.status_label = Label("pose input: starting...")

        self.items = (
            self.left_robot,
            self.right_robot,
            self.left_input,
            self.right_input,
            self.left_flange,
            self.right_flange,
            self.left_base,
            self.right_base,
            *(() if self.horizon_frame is None else (self.horizon_frame,)),
            self.status_label,
        )

        self.env = Swift()
        self.env.launch(realtime=True)
        for item in self.items:
            self.env.add(item)
        self.logger.info("swift ready")

    def update(
        self,
        left_q: np.ndarray | None,
        right_q: np.ndarray | None,
        frames: CoordinateFrames,
        status: str,
        dt: float,
    ) -> None:
        if left_q is not None:
            self.left_robot.q = left_q
        if right_q is not None:
            self.right_robot.q = right_q

        if frames.left is not None:
            self.left_input.T = swift_se3(frames.left.transform_horizon_to_controller, self.z_offset_m).A
            self.left_flange.T = swift_se3(frames.left.transform_horizon_to_flange, self.z_offset_m).A
        if frames.right is not None:
            self.right_input.T = swift_se3(frames.right.transform_horizon_to_controller, self.z_offset_m).A
            self.right_flange.T = swift_se3(frames.right.transform_horizon_to_flange, self.z_offset_m).A

        self.status_label.desc = status
        self.env.step(dt)

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:
            pass
