from __future__ import annotations
import logging
import time
import numpy as np
from spatialgeometry import Axes
from spatialmath import SE3
from config import MotionConfig, SimConfig
from frames import ArmBases, CoordinateFrames
from motion import joint_trajectory
_SWIFT_FROM_HORIZON = np.eye(4)
_SWIFT_FROM_HORIZON[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
def swift_se3(transform_horizon: np.ndarray, z_offset_m: float = 0.0) -> SE3:
    transform = _SWIFT_FROM_HORIZON @ transform_horizon
    transform[:3, 3] += np.array([0.0, 0.0, z_offset_m])
    return SE3.Rt(transform[:3, :3], transform[:3, 3])
class SwiftSimBackend:
    def __init__(
        self, left_robot, right_robot, bases: ArmBases, sim_config: SimConfig, motion_config: MotionConfig, logger: logging.Logger
    ) -> None:
        from swift import Swift
        from swift.SwiftElement import Label
        self.logger = logger
        self.motion_config = motion_config
        self.z_offset_m = sim_config.visual_z_offset_m
        self.joint_log_period_s = max(0.0, float(sim_config.joint_log_period_s))
        self.next_joint_log_t = 0.0
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
    def move_j_startup(self, side: str, q: np.ndarray) -> None:
        dt = 1.0 / self.motion_config.command_hz
        empty_frames = CoordinateFrames(None, None)
        start = self.current_joints(side)
        for q_cmd in joint_trajectory(self.motion_config, start, q):
            left_q, right_q = (q_cmd, None) if side == "left" else (None, q_cmd)
            self.update(left_q, right_q, empty_frames, f"moving {side} arm to startup pose", dt)
            time.sleep(dt)
        self.status_label.desc = "startup pose reached; waiting for Enter"
    def prepare_teleop(self, side: str) -> np.ndarray:
        return self.current_joints(side)
    def current_joints(self, side: str, wait: bool = False) -> np.ndarray:
        robot = self.left_robot if side == "left" else self.right_robot if side == "right" else None
        if robot is None:
            raise ValueError(f"unsupported arm side: {side}")
        return np.asarray(robot.q, dtype=float).copy()
    def update(self, left_q: np.ndarray | None, right_q: np.ndarray | None, frames: CoordinateFrames, status: str, dt: float) -> None:
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
        self._log_joint_angles()
        self.env.step(dt)
    def _log_joint_angles(self) -> None:
        if self.joint_log_period_s <= 0.0:
            return
        now = time.monotonic()
        if now < self.next_joint_log_t:
            return
        self.next_joint_log_t = now + self.joint_log_period_s
        left = np.array2string(np.asarray(self.left_robot.q, dtype=float), precision=4, suppress_small=True)
        right = np.array2string(np.asarray(self.right_robot.q, dtype=float), precision=4, suppress_small=True)
        self.logger.info("sim joints left=%s right=%s", left, right)
    def diagnostics(self) -> None:
        return None
    def close(self) -> None:
        try:
            self.env.close()
        except Exception:
            pass
