from __future__ import annotations

import logging
import time

import numpy as np

from backends import HardwareBackend
from config import ControlConfig
from filters import PoseFilter
from frames import arm_base_in_horizon, build_coordinate_frames
from ik import NeroIK
from motion import ArmMotionPlanner
from pose_io import PoseInputSubscriber
from sim import SwiftSimBackend


def direct_joint_command(q_target: np.ndarray | None, q_last: np.ndarray) -> np.ndarray:
    return q_target.copy() if q_target is not None else q_last.copy()


def run_control(config: ControlConfig, logger: logging.Logger) -> int:
    if config.runtime.mode != "dual_sim":
        HardwareBackend(config.runtime.mode)
        return 1

    pose_input = PoseInputSubscriber(config.pose_input)
    left_ik = NeroIK(config.ik)
    right_ik = NeroIK(config.ik)
    bases = arm_base_in_horizon(config.frames)
    backend = SwiftSimBackend(left_ik.robot, right_ik.robot, bases, config.sim, logger)

    left_filter = PoseFilter(config.filters.pose)
    right_filter = PoseFilter(config.filters.pose)
    left_command = left_ik.robot.q.copy()
    right_command = right_ik.robot.q.copy()
    left_motion = ArmMotionPlanner(config.motion, left_command) if config.motion.enabled else None
    right_motion = ArmMotionPlanner(config.motion, right_command) if config.motion.enabled else None

    dt = 1.0 / config.motion.command_hz
    n = 0
    last_status = ""
    last_hold_log = [0.0, 0.0]
    try:
        while True:
            tick_start = time.monotonic()
            pose_input.spin()
            now = time.monotonic()
            left_pose = left_filter.apply(pose_input.get_left(), now)
            right_pose = right_filter.apply(pose_input.get_right(), now)
            frames = build_coordinate_frames(left_pose, right_pose, config.frames)

            left_target = None
            right_target = None
            if frames.left is not None:
                left_target = left_ik.solve(frames.left.transform_arm_base_to_flange)
                if left_target is None and now - last_hold_log[0] > 1.0:
                    logger.info("IK hold: left")
                    last_hold_log[0] = now
            if frames.right is not None:
                right_target = right_ik.solve(frames.right.transform_arm_base_to_flange)
                if right_target is None and now - last_hold_log[1] > 1.0:
                    logger.info("IK hold: right")
                    last_hold_log[1] = now

            if config.motion.enabled:
                left_q = left_motion.step(left_target, now)
                right_q = right_motion.step(right_target, now)
            else:
                left_command = direct_joint_command(left_target, left_command)
                right_command = direct_joint_command(right_target, right_command)
                left_q = left_command.copy()
                right_q = right_command.copy()

            connected = pose_input.connected(config.pose_input.disconnect_s)
            if not connected:
                status = "pose input: disconnected (waiting...)"
            else:
                lt = "ok" if pose_input.tracked[0] == 1 else "lost"
                rt = "ok" if pose_input.tracked[1] == 1 else "lost"
                status = f"pose input: connected | L {lt} | R {rt}"
            if status != last_status:
                logger.info(status)
                last_status = status

            n += 1
            if n % int(config.motion.command_hz) == 0 and frames.left is not None:
                in_pos = frames.left.transform_horizon_to_controller[:3, 3]
                flange = frames.left.transform_horizon_to_flange[:3, 3]
                logger.info("L input=%s flange=%s", np.round(in_pos, 4), np.round(flange, 4))

            backend.update(left_q, right_q, frames, status, dt)
            elapsed = time.monotonic() - tick_start
            time.sleep(max(0.0, dt - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        backend.close()
        pose_input.close()
    return 0
