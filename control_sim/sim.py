from __future__ import annotations

import time

import numpy as np
from spatialmath import SE3

from .config import (
    FrameTransformConfig,
    IkConfig,
    RobotControlConfig,
    RosConfig,
    SimConfig,
)
from .frames import arm_base_in_horizon, build_coordinate_frames
from .ik import NeroIK
from .ros_io import PicoPoseSubscriber


class NoopRobotController:
    """Reserved hook for real-robot control. send() does nothing for now."""

    def send(self, left_q: np.ndarray | None, right_q: np.ndarray | None) -> None:
        pass

    def close(self) -> None:
        pass


def _se3_from_numpy(T: np.ndarray) -> SE3:
    return SE3.Rt(T[:3, :3], T[:3, 3])


def _make_marker(radius_m: float, color: str):
    from spatialgeometry import Sphere

    marker = Sphere(radius_m)
    try:
        marker.color = color
    except Exception:
        pass
    return marker


def run_sim(
    ros_config: RosConfig,
    frame_config: FrameTransformConfig,
    ik_config: IkConfig,
    sim_config: SimConfig,
    control_config: RobotControlConfig,
) -> int:
    from swift import Swift

    sub = PicoPoseSubscriber(ros_config)
    left_ik = NeroIK(ik_config, frame_config)
    right_ik = NeroIK(ik_config, frame_config)
    left_marker = _make_marker(sim_config.marker_radius_m, "#ff5500")
    right_marker = _make_marker(sim_config.marker_radius_m, "#00aaff")

    bases = arm_base_in_horizon(frame_config)
    left_ik.robot.base = _se3_from_numpy(bases.left)
    right_ik.robot.base = _se3_from_numpy(bases.right)

    env = Swift()
    env.launch(realtime=True)
    env.add(left_ik.robot)
    env.add(right_ik.robot)
    env.add(left_marker)
    env.add(right_marker)

    controller = NoopRobotController()
    dt = 1.0 / sim_config.step_hz
    try:
        while True:
            sub.spin()
            frames = build_coordinate_frames(sub.get_left(), sub.get_right(), frame_config)

            left_q = None
            right_q = None
            if frames.left is not None:
                q = left_ik.solve(frames.left.transform_arm_base_from_flange)
                left_ik.robot.q = q
                left_q = q
                left_marker.T = _se3_from_numpy(frames.left.transform_pico_horizon_from_flange).A
            if frames.right is not None:
                q = right_ik.solve(frames.right.transform_arm_base_from_flange)
                right_ik.robot.q = q
                right_q = q
                    right_q = q
                right_marker.T = _se3_from_numpy(frames.right.transform_pico_horizon_from_flange).A

            if control_config.enabled:
                controller.send(left_q, right_q)

            env.step(dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        sub.close()
    return 0
