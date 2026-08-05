from __future__ import annotations

import logging
import sys
import time
from pathlib import Path
from types import MethodType
from typing import Callable

import numpy as np

from config import MotionConfig, RealArmConfig
from frames import CoordinateFrames
from motion import max_abs_angle_diff, plan_joint_trajectory


def _sdk(config: RealArmConfig):
    sdk_path = Path(config.sdk_path)
    if not sdk_path.is_absolute():
        sdk_path = Path(__file__).resolve().parent / sdk_path
    if sdk_path.exists() and str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

    return AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


def _firmware_constant(config: RealArmConfig, firmware: str):
    _, _, NeroFW, _ = _sdk(config)
    if firmware == "v120":
        return NeroFW.V120
    if firmware == "v112":
        return NeroFW.V112
    if firmware == "v111":
        return NeroFW.V111
    return NeroFW.DEFAULT


def _parse_version_tuple(version: str) -> tuple[int, ...]:
    parts = []
    for part in version.split("."):
        digits = "".join(ch for ch in part if ch.isdigit())
        if not digits:
            break
        parts.append(int(digits))
    return tuple(parts)


def ensure_socketcan_ready(config: RealArmConfig) -> None:
    if config.interface != "socketcan":
        return
    net_dir = Path("/sys/class/net") / config.channel
    if not net_dir.exists():
        raise RuntimeError(f"SocketCAN interface {config.channel!r} does not exist")
    flags = int((net_dir / "flags").read_text(encoding="utf-8").strip(), 16)
    if not flags & 0x1:
        raise RuntimeError(
            f"SocketCAN interface {config.channel!r} is DOWN. Run: "
            f"sudo ip link set {config.channel} type can bitrate {config.bitrate}; "
            f"sudo ip link set {config.channel} up"
        )


def create_robot(config: RealArmConfig, firmware: str):
    AgxArmFactory, ArmModel, _, create_agx_arm_config = _sdk(config)
    cfg = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=_firmware_constant(config, firmware),
        interface=config.interface,
        channel=config.channel,
        bitrate=config.bitrate,
    )
    robot = AgxArmFactory.create_arm(cfg)
    try:
        robot.connect()
    except Exception:
        robot.disconnect()
        raise
    return robot


def detect_firmware(config: RealArmConfig, logger: logging.Logger) -> str:
    robot = create_robot(config, "default")
    deadline = time.monotonic() + config.enable_timeout_s
    last_error = None
    try:
        while time.monotonic() < deadline:
            try:
                fw = robot.get_firmware(timeout=0.3, min_interval=0.0)
                if fw:
                    software = str(fw.get("software_version", "")).strip()
                    version = _parse_version_tuple(software)
                    driver = "v120" if version >= (1, 20) else "v112" if version >= (1, 12) else "v111" if version >= (1, 11) else "default"
                    logger.info("detected Nero firmware=%s driver=%s", software, driver)
                    return driver
                robot.enable()
            except Exception as exc:
                last_error = exc
            time.sleep(0.05)
    finally:
        robot.disconnect()
    raise RuntimeError("timed out detecting Nero firmware; check CAN feedback or set RealArmConfig.firmware explicitly") from last_error


def install_can_send_timeout(robot, timeout_s: float | None) -> None:
    if timeout_s is None:
        return
    comm = robot.get_context().get_comm()
    if comm is None:
        raise RuntimeError("robot comm is not initialized")
    original_send = getattr(comm, "_analytic_control_original_send", None)
    if original_send is None:
        original_send = comm.send
        comm._analytic_control_original_send = original_send

    def send_with_default_timeout(msg, timeout=None):
        return original_send(msg, timeout_s if timeout is None else timeout)

    comm.send = send_with_default_timeout


def install_can_frame_interval(robot, interval_s: float) -> None:
    if interval_s <= 0.0 or not hasattr(robot, "_send_msg"):
        return

    def delayed_send_msgs(self, msgs):
        for index, msg in enumerate(msgs):
            self._send_msg(msg)
            if index + 1 < len(msgs):
                time.sleep(interval_s)

    robot._send_msgs = MethodType(delayed_send_msgs, robot)


def prepare_motion_mode(robot, mode: str) -> None:
    robot.set_motion_mode({"move_j": "j", "move_js": "js"}[mode])
    robot.set_auto_set_motion_mode_enabled(False)


def read_joint_angles_optional(robot) -> list[float] | None:
    msg = robot.get_joint_angles()
    if msg is None or msg.msg is None:
        return None
    return list(msg.msg)


def read_current_joint_angles(robot, timeout_s: float) -> np.ndarray:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        values = read_joint_angles_optional(robot)
        if values is not None:
            return np.asarray(values, dtype=float).reshape(7)
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for live joint feedback")


def wait_for_feedback(robot, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if read_joint_angles_optional(robot) is not None or robot.get_arm_status() is not None:
            return True
        time.sleep(0.02)
    return False


def enable_normal_push(robot, timeout_s: float, firmware: str) -> bool:
    uses_new_can_push = firmware in {"v112", "v120"}
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if not uses_new_can_push:
                robot.set_normal_mode()
            if robot.enable():
                if not uses_new_can_push:
                    robot.set_normal_mode()
                return True
        except Exception:
            pass
        time.sleep(0.05)
    return False


def validate_joint_command(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(7)
    if not np.all(np.isfinite(q)):
        raise ValueError("joint command contains NaN or Inf")
    return q


class RealArmBackend:
    def __init__(
        self,
        side: str,
        real_config: RealArmConfig,
        motion_config: MotionConfig,
        logger: logging.Logger,
        robot_factory: Callable[[RealArmConfig, logging.Logger], object] | None = None,
        diagnostics=None,
        pose_sampler: Callable[[], object] | None = None,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError(f"unsupported real arm side: {side}")
        self.side = side
        self.config = real_config
        self.motion_config = motion_config
        self.logger = logger
        self.diagnostics = diagnostics
        self.pose_sampler = pose_sampler
        self.firmware = real_config.firmware
        self.robot = robot_factory(real_config, logger) if robot_factory else self._connect_robot()
        self.q_feedback = read_current_joint_angles(self.robot, real_config.startup_timeout_s)
        self.last_sent_q = self.q_feedback.copy()
        self.motor_feedback_valid: bool | None = None
        self.motor_timestamps: np.ndarray | None = None
        self.motor_updated_at = np.zeros(7, dtype=float)
        self.sample_feedback()

    def _connect_robot(self):
        ensure_socketcan_ready(self.config)
        firmware = detect_firmware(self.config, self.logger) if self.config.firmware == "auto" else self.config.firmware
        self.firmware = firmware
        robot = create_robot(self.config, firmware)
        install_can_send_timeout(robot, self.config.can_send_timeout_s)
        install_can_frame_interval(robot, self.config.can_frame_interval_s)
        if not enable_normal_push(robot, self.config.enable_timeout_s, firmware):
            robot.disconnect()
            raise RuntimeError("timed out while enabling/opening CAN feedback push")
        robot.set_joint_limits_enabled(self.config.sdk_joint_limits_enabled)
        robot.set_speed_percent(self.config.speed_percent)
        if not wait_for_feedback(robot, self.config.startup_timeout_s):
            robot.disconnect()
            raise RuntimeError("timed out waiting for live robot feedback")
        self.logger.info("connected Nero real arm side=%s channel=%s firmware=%s", self.side, self.config.channel, firmware)
        return robot

    def move_j_startup(self, side: str, q: np.ndarray) -> None:
        if side != self.side:
            raise ValueError(f"{side} arm is not connected")
        target = np.asarray(q, dtype=float).reshape(7)
        if max_abs_angle_diff(target, self.q_feedback) <= 1e-6:
            return
        prepare_motion_mode(self.robot, "move_j")
        for point in plan_joint_trajectory(self.motion_config, self.q_feedback, target):
            point = validate_joint_command(point)
            self.robot.move_j([float(v) for v in point])
            self.last_sent_q = point.copy()
            self._record_command("startup", "move_j", point)
            self.record_feedback("startup")
            time.sleep(1.0 / self.motion_config.command_hz)
        feedback = self.sample_feedback()
        self.q_feedback = feedback["q"].copy() if feedback["fresh"] else target.copy()
        self.last_sent_q = target.copy()

    def _cached_feedback(self) -> dict:
        values = read_joint_angles_optional(self.robot)
        if values is not None:
            self.q_feedback = validate_joint_command(values)
        return {"q": self.q_feedback.copy(), "fresh": False, "source": "joint_angles_cache"}

    def sample_feedback(self) -> dict:
        packets = []
        try:
            packets = [self.robot.get_motor_states(index) for index in range(1, 8)]
        except Exception:
            return self._cached_feedback()
        if any(packet is None or packet.msg is None for packet in packets):
            return self._cached_feedback()

        fields = {
            name: np.asarray([float(getattr(packet.msg, name)) for packet in packets])
            for name in ("position", "velocity", "current", "torque")
        }
        timestamps = np.asarray([float(packet.timestamp) for packet in packets])
        if not all(np.all(np.isfinite(value)) for value in (*fields.values(), timestamps)) or np.any(timestamps <= 0.0):
            return self._cached_feedback()

        q = fields["position"]
        if self.motor_feedback_valid is None:
            delta = max_abs_angle_diff(q, self.q_feedback)
            self.motor_feedback_valid = delta <= self.config.feedback_validation_max_delta_rad
            if not self.motor_feedback_valid:
                self.logger.warning("motor feedback rejected: joint-angle mismatch %.4frad", delta)
        if not self.motor_feedback_valid:
            return self._cached_feedback()

        now = time.monotonic()
        if self.motor_timestamps is None:
            self.motor_updated_at[:] = now
        else:
            self.motor_updated_at[timestamps != self.motor_timestamps] = now
        self.motor_timestamps = timestamps.copy()
        self.q_feedback = q.copy()
        return {
            "q": q, "velocity": fields["velocity"], "current": fields["current"],
            "torque": fields["torque"], "timestamps": timestamps,
            "fresh": bool(np.all(now - self.motor_updated_at <= self.config.feedback_stale_s)),
            "source": "motor_states",
        }

    def _record_command(self, phase: str, mode: str, q: np.ndarray) -> None:
        if self.diagnostics is not None:
            self.diagnostics.record(phase, event="command_sent", command_mode=mode, command_q=q)

    def record_feedback(self, phase: str) -> dict:
        feedback = self.sample_feedback()
        if self.diagnostics is not None:
            pico = self.pose_sampler() if self.pose_sampler is not None else None
            self.diagnostics.record(phase, event="feedback", pico_raw=pico, feedback=feedback)
        return feedback

    def prepare_teleop(self) -> np.ndarray:
        feedback = self.sample_feedback()
        hold_q = feedback["q"].copy() if feedback["fresh"] else self.last_sent_q.copy()
        prepare_motion_mode(self.robot, "move_js")
        dt = 1.0 / self.motion_config.command_hz
        for _ in range(max(1, int(np.ceil(self.config.servo_handoff_s / dt)))):
            self.robot.move_js([float(v) for v in hold_q])
            self.last_sent_q = hold_q.copy()
            self._record_command("handoff", "move_js", hold_q)
            self.record_feedback("handoff")
            time.sleep(dt)
        self.logger.info("servo handoff complete side=%s hold_q=%s", self.side, np.round(hold_q, 6))
        return hold_q

    def update(
        self,
        left_q: np.ndarray | None,
        right_q: np.ndarray | None,
        frames: CoordinateFrames,
        status: str,
        dt: float,
    ) -> dict:
        feedback = self.sample_feedback()
        q = left_q if self.side == "left" else right_q
        if q is None:
            return feedback
        q = validate_joint_command(q)
        self.robot.move_js([float(v) for v in q])
        self.last_sent_q = q.copy()
        self._record_command("teleop", "move_js", q)
        return feedback

    def current_joints(self) -> np.ndarray:
        return self.sample_feedback()["q"].copy()

    def close(self) -> None:
        try:
            try:
                self.robot.set_auto_set_motion_mode_enabled(True)
            except Exception:
                pass
            if self.firmware not in {"v112", "v120"}:
                self.robot.set_normal_mode()
        finally:
            self.robot.disconnect()
