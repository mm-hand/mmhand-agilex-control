from __future__ import annotations
import logging
import sys
import time
from pathlib import Path
import numpy as np
from config import MotionConfig, RealArmConfig
from frames import CoordinateFrames
def _sdk(config: RealArmConfig):
    sdk_path = Path(config.sdk_path)
    if not sdk_path.is_absolute():
        sdk_path = Path(__file__).resolve().parent / sdk_path
    if sdk_path.exists() and str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
    return AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
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
def create_robot(config: RealArmConfig):
    AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config = _sdk(config)
    cfg = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=NeroFW.DEFAULT,
        interface=config.interface,
        channel=config.channel,
        bitrate=config.bitrate,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()
    return robot
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
def enable_normal_push(robot, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if hasattr(robot, "set_normal_mode"):
                robot.set_normal_mode()
            if robot.enable():
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
    def __init__(self, side: str, real_config: RealArmConfig, motion_config: MotionConfig, logger: logging.Logger) -> None:
        if side not in {"left", "right"}:
            raise ValueError(f"unsupported real arm side: {side}")
        self.side = side
        self.config = real_config
        self.motion_config = motion_config
        self.logger = logger
        self.robot = self._connect_robot()
        self.q_feedback = read_current_joint_angles(self.robot, real_config.startup_timeout_s)
        self._set_servo_mode()
        self.last_sent_q = self.q_feedback.copy()
    def _connect_robot(self):
        ensure_socketcan_ready(self.config)
        robot = create_robot(self.config)
        try:
            if not enable_normal_push(robot, self.config.enable_timeout_s):
                raise RuntimeError("timed out while enabling/opening CAN feedback push")
            if hasattr(robot, "set_joint_limits_enabled"):
                robot.set_joint_limits_enabled(self.config.sdk_joint_limits_enabled)
            if hasattr(robot, "set_speed_percent"):
                robot.set_speed_percent(self.config.speed_percent)
            self.logger.info("connected Nero real arm side=%s channel=%s", self.side, self.config.channel)
            return robot
        except Exception:
            robot.disconnect()
            raise
    def _set_servo_mode(self) -> None:
        if hasattr(self.robot, "set_motion_mode"):
            self.robot.set_motion_mode("js")
        if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
            self.robot.set_auto_set_motion_mode_enabled(False)
    def move_j_startup(self, q: np.ndarray) -> None:
        q = validate_joint_command(q)
        if hasattr(self.robot, "set_motion_mode"):
            self.robot.set_motion_mode("j")
        self.robot.move_j([float(v) for v in q])
        feedback = read_joint_angles_optional(self.robot)
        self.q_feedback = np.asarray(feedback, dtype=float).reshape(7) if feedback is not None else q.copy()
        self._set_servo_mode()
        self.last_sent_q = self.q_feedback.copy()
    def update(self, left_q: np.ndarray | None, right_q: np.ndarray | None, frames: CoordinateFrames, status: str, dt: float) -> None:
        q = left_q if self.side == "left" else right_q
        if q is None:
            return
        q = validate_joint_command(q)
        self.robot.move_js([float(v) for v in q])
        self.last_sent_q = q.copy()
    def close(self) -> None:
        try:
            try:
                if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
                    self.robot.set_auto_set_motion_mode_enabled(True)
                if hasattr(self.robot, "set_normal_mode"):
                    self.robot.set_normal_mode()
            except Exception:
                pass
        finally:
            self.robot.disconnect()
