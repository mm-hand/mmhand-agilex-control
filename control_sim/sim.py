from __future__ import annotations

import time

import numpy as np
from spatialmath import SE3
from spatialgeometry import Axes

from .config import FrameTransformConfig, IkConfig, RobotControlConfig, RosConfig, SimConfig
from .frames import arm_base_in_horizon, build_coordinate_frames
from .ik import NeroIK
from .ros_io import PicoPoseSubscriber

# Swift world is Z-up; PICO horizon is Y-up. Convert: Swift_X=hX, Swift_Y=-hZ, Swift_Z=hY
_S = np.eye(4)
_S[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])


class NoopRobotController:
    def send(self, left_q, right_q): pass
    def close(self): pass


def _s2se3(T):
    return SE3.Rt((_S @ T)[:3, :3], (_S @ T)[:3, 3])


def run_sim(ros_cfg, frame_cfg, ik_cfg, sim_cfg, ctrl_cfg):
    from swift import Swift

    sub = PicoPoseSubscriber(ros_cfg)
    L, R = NeroIK(ik_cfg), NeroIK(ik_cfg)
    bases = arm_base_in_horizon(frame_cfg)
    L.robot.base = _s2se3(bases.left)
    R.robot.base = _s2se3(bases.right)

    lc, rc, lf, rf, lb, rb = (Axes(0.05) for _ in range(6))
    lb.T = _s2se3(bases.left).A
    rb.T = _s2se3(bases.right).A
    env = Swift()
    env.launch(realtime=True)
    for o in (L.robot, R.robot, lc, rc, lf, rf, lb, rb):
        env.add(o)
    print("swift ready", flush=True)

    dt = 1.0 / sim_cfg.step_hz
    n = 0
    try:
        while True:
            sub.spin()
            f = build_coordinate_frames(sub.get_left(), sub.get_right(), frame_cfg)
            lq = rq = None
            if f.left is not None:
                q = L.solve(f.left.transform_arm_base_from_flange)
                if q is not None:
                    L.robot.q = q; lq = q
                else:
                    print("[IK] L hold", flush=True)
                lc.T = _s2se3(f.left.transform_pico_horizon_from_pico_controller).A
                lf.T = _s2se3(f.left.transform_pico_horizon_from_flange).A
            if f.right is not None:
                q = R.solve(f.right.transform_arm_base_from_flange)
                if q is not None:
                    R.robot.q = q; rq = q
                else:
                    print("[IK] R hold", flush=True)
                rc.T = _s2se3(f.right.transform_pico_horizon_from_pico_controller).A
                rf.T = _s2se3(f.right.transform_pico_horizon_from_flange).A
            if ctrl_cfg.enabled:
                NoopRobotController().send(lq, rq)
            n += 1
            if n % 50 == 0 and f.left is not None:
                ch = f.left.transform_pico_horizon_from_pico_controller[:3, 3]
                fh = f.left.transform_pico_horizon_from_flange[:3, 3]
                print(f"[L] h ctrl={np.round(ch,4)} flange={np.round(fh,4)}", flush=True)
                print(f"[L] s ctrl={np.round((_S@f.left.transform_pico_horizon_from_pico_controller)[:3,3],4)} flange={np.round((_S@f.left.transform_pico_horizon_from_flange)[:3,3],4)}", flush=True)
            env.step(dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
    return 0
