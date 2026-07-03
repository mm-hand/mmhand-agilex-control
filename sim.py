from __future__ import annotations

import time

import numpy as np
from spatialmath import SE3
from spatialgeometry import Axes

from config import FrameTransformConfig, IkConfig, RosConfig, SimConfig
from frames import arm_base_in_horizon, build_coordinate_frames
from ik import NeroIK
from ros_io import PicoPoseSubscriber

# Swift world is Z-up; PICO horizon is Y-up.
# Swift_X = hX, Swift_Y = -hZ, Swift_Z = hY.
_SWIFT_FROM_HORIZON = np.eye(4)
_SWIFT_FROM_HORIZON[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])


def _swift_se3(transform_horizon: np.ndarray) -> SE3:
    transform = _SWIFT_FROM_HORIZON @ transform_horizon
    return SE3.Rt(transform[:3, :3], transform[:3, 3])


def run_sim(
    ros_cfg: RosConfig,
    frame_cfg: FrameTransformConfig,
    ik_cfg: IkConfig,
    sim_cfg: SimConfig,
) -> int:
    from swift import Swift

    sub = PicoPoseSubscriber(ros_cfg)
    left_ik = NeroIK(ik_cfg)
    right_ik = NeroIK(ik_cfg)

    bases = arm_base_in_horizon(frame_cfg)
    left_ik.robot.base = _swift_se3(bases.left)
    right_ik.robot.base = _swift_se3(bases.right)

    left_controller, right_controller, left_flange, right_flange, left_base, right_base = (
        Axes(0.05) for _ in range(6)
    )
    left_base.T = _swift_se3(bases.left).A
    right_base.T = _swift_se3(bases.right).A

    env = Swift()
    env.launch(realtime=True)
    for item in (
        left_ik.robot,
        right_ik.robot,
        left_controller,
        right_controller,
        left_flange,
        right_flange,
        left_base,
        right_base,
    ):
        env.add(item)
    print("swift ready", flush=True)

    dt = 1.0 / sim_cfg.step_hz
    n = 0
    try:
        while True:
            sub.spin()
            frames = build_coordinate_frames(sub.get_left(), sub.get_right(), frame_cfg)

            if frames.left is not None:
                q = left_ik.solve(frames.left.transform_arm_base_to_flange)
                if q is not None:
                    left_ik.robot.q = q
                else:
                    print("[IK] L hold", flush=True)
                left_controller.T = _swift_se3(frames.left.transform_horizon_to_controller).A
                left_flange.T = _swift_se3(frames.left.transform_horizon_to_flange).A

            if frames.right is not None:
                q = right_ik.solve(frames.right.transform_arm_base_to_flange)
                if q is not None:
                    right_ik.robot.q = q
                else:
                    print("[IK] R hold", flush=True)
                right_controller.T = _swift_se3(frames.right.transform_horizon_to_controller).A
                right_flange.T = _swift_se3(frames.right.transform_horizon_to_flange).A

            n += 1
            if n % 50 == 0 and frames.left is not None:
                ctrl = frames.left.transform_horizon_to_controller[:3, 3]
                flange = frames.left.transform_horizon_to_flange[:3, 3]
                print(f"[L] h ctrl={np.round(ctrl, 4)} flange={np.round(flange, 4)}", flush=True)

            env.step(dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
    return 0
