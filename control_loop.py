from __future__ import annotations

import json
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


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    return value


def _ik_event(report: dict[str, object], config: ControlConfig) -> tuple[str | None, tuple[object, ...]]:
    if not bool(report.get("ok", False)):
        return (
            "failure",
            (
                report.get("failure_reason"),
                report.get("method"),
                report.get("selected_by"),
                report.get("branch_prev"),
            ),
        )
    solve_ms = float(report.get("solve_ms", 0.0) or 0.0)
    if solve_ms > config.log.ik_slow_solve_ms:
        return "slow", (round(solve_ms, 1), report.get("method"), report.get("selected_by"))
    clamp = report.get("target_clamp", {})
    if config.log.ik_log_target_clamp and isinstance(clamp, dict) and clamp.get("clamped"):
        return "target_clamp", (clamp.get("reason"), round(float(clamp.get("wrist_dist_before_m", 0.0)), 4))
    return None, ()


def _log_ik_report(
    logger: logging.Logger,
    config: ControlConfig,
    arm: str,
    report: dict[str, object],
    now: float,
    state: dict[str, object],
) -> None:
    if not config.log.ik_diagnostics_enabled:
        return
    event, key = _ik_event(report, config)
    if event is None:
        return
    interval = config.log.ik_failure_log_interval_s if event == "failure" else 1.0
    last_time = float(state.get("time", 0.0))
    last_key = state.get("key")
    if key == last_key and now - last_time < interval:
        return
    state["time"] = now
    state["key"] = key
    payload = {"event": event, "arm": arm, "ik": report}
    logger.info("IK diagnostic: %s", json.dumps(_json_safe(payload), ensure_ascii=False, sort_keys=True))


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
    last_ik_log = [{"time": 0.0, "key": None}, {"time": 0.0, "key": None}]
    last_overrun_log = 0.0
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
                left_target, left_report = left_ik.solve_with_report(frames.left.transform_arm_base_to_flange)
                _log_ik_report(logger, config, "left", left_report, now, last_ik_log[0])
            if frames.right is not None:
                right_target, right_report = right_ik.solve_with_report(frames.right.transform_arm_base_to_flange)
                _log_ik_report(logger, config, "right", right_report, now, last_ik_log[1])

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
            if elapsed > dt and now - last_overrun_log > 1.0:
                last_overrun_log = now
                logger.info(
                    "control loop overrun: %s",
                    json.dumps(
                        {"elapsed_ms": round(elapsed * 1000.0, 3), "budget_ms": round(dt * 1000.0, 3)},
                        sort_keys=True,
                    ),
                )
            time.sleep(max(0.0, dt - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        backend.close()
        pose_input.close()
    return 0
