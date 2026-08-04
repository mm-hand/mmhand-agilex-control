from __future__ import annotations
import logging
import time
import numpy as np
from config import ControlConfig
from filters import JointTargetFilter, PoseFilter, PoseInputLimiter
from frames import arm_base_in_horizon, build_coordinate_frames
from ik import NeroIK
from motion import ArmMotionPlanner
from pose_io import PoseInputSubscriber
from run_log import SessionLogger
from sim import SwiftSimBackend
def _real_side(mode: str) -> str | None:
    if mode == "left_real":
        return "left"
    if mode == "right_real":
        return "right"
    if mode == "dual_sim":
        return None
    raise ValueError(f"unknown runtime mode: {mode}")
def _startup_pose(config: ControlConfig, side: str) -> np.ndarray:
    return np.asarray(getattr(config.startup, f"{side}_pose_rad"), dtype=float).reshape(7)
def _motion_step(planner: ArmMotionPlanner, target: np.ndarray | None, now: float) -> np.ndarray | None:
    if target is None:
        planner.step(None, now)
        return None
    return planner.step(target, now)
def _flange_in_horizon(ik: NeroIK, horizon_to_arm_base: np.ndarray, q: np.ndarray | None) -> np.ndarray | None:
    if q is None:
        return None
    return horizon_to_arm_base @ ik.fk_flange(q)
def _record_side(
    session: SessionLogger,
    side: str,
    connected: bool,
    tracked: bool,
    raw_pose,
    filtered_pose,
    frame,
    current_q,
    ik_q,
    filtered_q,
    command_q,
    current_flange,
    ik_flange,
    command_flange,
    diagnostics,
) -> None:
    session.record(
        "teleop", side, connected=connected, tracked=tracked,
        pico_raw=raw_pose, pico_filtered=filtered_pose,
        target_flange=None if frame is None else frame.transform_horizon_to_flange,
        current_q=current_q, ik_q=ik_q, filtered_q=filtered_q, command_q=command_q,
        current_flange=current_flange, ik_flange=ik_flange, command_flange=command_flange,
        diagnostics=diagnostics,
    )
def run_control(config: ControlConfig, logger: logging.Logger, session: SessionLogger) -> int:
    pose_input = PoseInputSubscriber(config.pose_input)
    backend = None
    try:
        left_ik = NeroIK(config.ik)
        right_ik = NeroIK(config.ik)
        real_side = _real_side(config.runtime.mode)
        bases = arm_base_in_horizon(config.frames)
        use_left = real_side is None or real_side == "left"
        use_right = real_side is None or real_side == "right"
        if real_side is None:
            backend = SwiftSimBackend(left_ik.robot, right_ik.robot, bases, config.sim, config.motion, logger)
        else:
            from hardware import RealArmBackend
            backend = RealArmBackend(real_side, config.real, config.motion, logger)
        if config.startup.enable_startup_pose:
            if use_left:
                startup = _startup_pose(config, "left")
                session.event("startup_move", "left", command_q=startup)
                backend.move_j_startup("left", startup)
                session.event("startup_reached", "left", current_q=backend.current_joints("left"))
            if use_right:
                startup = _startup_pose(config, "right")
                session.event("startup_move", "right", command_q=startup)
                backend.move_j_startup("right", startup)
                session.event("startup_reached", "right", current_q=backend.current_joints("right"))
            input("startup pose reached; press Enter to start teleop...")
        left_start = backend.prepare_teleop("left") if use_left else left_ik.robot.q.copy()
        right_start = backend.prepare_teleop("right") if use_right else right_ik.robot.q.copy()
        if use_left:
            session.event("servo_hold", "left", current_q=left_start, command_q=left_start)
        if use_right:
            session.event("servo_hold", "right", current_q=right_start, command_q=right_start)
        if use_left:
            left_ik.reset_state(left_start)
        if use_right:
            right_ik.reset_state(right_start)
        left_pose_filter = PoseFilter(config.filters.pose)
        right_pose_filter = PoseFilter(config.filters.pose)
        left_limiter = PoseInputLimiter(config.input_limit)
        right_limiter = PoseInputLimiter(config.input_limit)
        left_target_filter = JointTargetFilter(config.filters.ik_output)
        right_target_filter = JointTargetFilter(config.filters.ik_output)
        left_motion = ArmMotionPlanner(config.motion, left_start)
        right_motion = ArmMotionPlanner(config.motion, right_start)
        dt = 1.0 / config.motion.command_hz
        last_status = ""
        while True:
            tick_start = time.monotonic()
            pose_input.spin()
            now = time.monotonic()
            left_raw = pose_input.get_left() if use_left else None
            right_raw = pose_input.get_right() if use_right else None
            left_pose = left_limiter.apply(left_pose_filter.apply(left_raw, now), now) if use_left else None
            right_pose = right_limiter.apply(right_pose_filter.apply(right_raw, now), now) if use_right else None
            frames = build_coordinate_frames(left_pose, right_pose, config.frames)
            left_target_raw = left_ik.solve(frames.left.transform_arm_base_to_flange) if use_left and frames.left else None
            right_target_raw = right_ik.solve(frames.right.transform_arm_base_to_flange) if use_right and frames.right else None
            left_target = left_target_filter.apply(left_target_raw, now) if use_left else None
            right_target = right_target_filter.apply(right_target_raw, now) if use_right else None
            left_q = _motion_step(left_motion, left_target, now) if use_left else None
            right_q = _motion_step(right_motion, right_target, now) if use_right else None
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
            backend.update(left_q, right_q, frames, status, dt)
            left_current_q = backend.current_joints("left") if use_left else None
            right_current_q = backend.current_joints("right") if use_right else None
            left_current_flange = _flange_in_horizon(left_ik, bases.left, left_current_q)
            left_command_flange = _flange_in_horizon(left_ik, bases.left, left_q)
            right_current_flange = _flange_in_horizon(right_ik, bases.right, right_current_q)
            right_command_flange = _flange_in_horizon(right_ik, bases.right, right_q)
            pose_input.publish_telemetry(
                left_current_q, left_q, left_current_flange, left_command_flange,
                right_current_q, right_q, right_current_flange, right_command_flange,
            )
            diagnostics = backend.diagnostics()
            if use_left:
                _record_side(
                    session, "left", connected, pose_input.tracked[0] == 1,
                    left_raw, left_pose, frames.left, left_current_q, left_target_raw, left_target, left_q,
                    left_current_flange, _flange_in_horizon(left_ik, bases.left, left_target_raw),
                    left_command_flange, diagnostics if real_side == "left" else None,
                )
            if use_right:
                _record_side(
                    session, "right", connected, pose_input.tracked[1] == 1,
                    right_raw, right_pose, frames.right, right_current_q, right_target_raw, right_target, right_q,
                    right_current_flange, _flange_in_horizon(right_ik, bases.right, right_target_raw),
                    right_command_flange, diagnostics if real_side == "right" else None,
                )
            elapsed = time.monotonic() - tick_start
            time.sleep(max(0.0, dt - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        if backend is not None:
            backend.close()
        pose_input.close()
    return 0
