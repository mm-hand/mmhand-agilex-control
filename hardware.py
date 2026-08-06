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
from motion import joint_trajectory

_FIRMWARE_NAMES = {"default", "v111", "v112", "v120"}

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
    constants = {
        "default": NeroFW.DEFAULT,
        "v111": NeroFW.V111,
        "v112": NeroFW.V112,
        "v120": NeroFW.V120,
    }
    try:
        return constants[firmware]
    except KeyError as exc:
        raise ValueError(f"unsupported Nero firmware: {firmware}") from exc

def _firmware_name(version: str) -> str:
    try:
        parsed = tuple(int(part) for part in version.strip().split(".")[:2])
    except ValueError as exc:
        raise RuntimeError(f"invalid Nero firmware version: {version!r}") from exc
    if parsed >= (1, 20):
        return "v120"
    if parsed >= (1, 12):
        return "v112"
    if parsed >= (1, 11):
        return "v111"
    return "default"

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
    last_error: Exception | None = None
    try:
        while time.monotonic() < deadline:
            try:
                firmware = robot.get_firmware(timeout=0.3, min_interval=0.0)
                if firmware:
                    software = str(firmware.get("software_version", "")).strip()
                    selected = _firmware_name(software)
                    logger.info("detected Nero firmware=%s driver=%s", software, selected)
                    return selected
                robot.enable()
            except Exception as exc:
                last_error = exc
            time.sleep(0.05)
    finally:
        robot.disconnect()
    message = (
        "timed out detecting Nero firmware; check CAN feedback or set "
        "RealArmConfig.firmware to default/v111/v112/v120"
    )
    raise RuntimeError(message) from last_error

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
    if interval_s <= 0.0 or not hasattr(robot, "_send_msgs"):
        return
    def delayed_send_msgs(self, msgs):
        for index, msg in enumerate(msgs):
            self._send_msg(msg)
            if index + 1 < len(msgs):
                time.sleep(interval_s)
    robot._send_msgs = MethodType(delayed_send_msgs, robot)

def read_joint_feedback_optional(robot) -> np.ndarray | None:
    msg = robot.get_joint_angles()
    if msg is None or msg.msg is None:
        return None
    q = np.asarray(msg.msg, dtype=float).reshape(7)
    if not np.all(np.isfinite(q)):
        return None
    return q

def read_motor_feedback_optional(robot) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    try:
        packets = [robot.get_motor_states(joint) for joint in range(1, 8)]
    except Exception:
        return None
    if any(packet is None or packet.msg is None for packet in packets):
        return None
    q = np.asarray([packet.msg.position for packet in packets], dtype=float)
    timestamps = np.asarray([getattr(packet, "timestamp", 0.0) for packet in packets], dtype=float)
    hz = np.asarray([getattr(packet, "hz", 0.0) for packet in packets], dtype=float)
    if not all(np.all(np.isfinite(values)) for values in (q, timestamps, hz)):
        return None
    return q, timestamps, hz

def read_motor_feedback(robot, timeout_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    deadline = time.monotonic() + timeout_s
    previous = None
    while time.monotonic() < deadline:
        feedback = read_motor_feedback_optional(robot)
        if feedback is not None:
            q, timestamps, hz = feedback
            live = hz > 0.0
            if previous is not None:
                live |= timestamps != previous
            if np.all(live):
                return q, timestamps, hz
            previous = timestamps
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for live motor feedback")

def read_current_joint_angles(robot, timeout_s: float) -> np.ndarray:
    return read_motor_feedback(robot, timeout_s)[0]

def wait_for_feedback(robot, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if read_motor_feedback_optional(robot) is not None or robot.get_arm_status() is not None:
            return True
        time.sleep(0.02)
    return False

def enable_normal_push(robot, timeout_s: float, firmware: str) -> None:
    modern = firmware in {"v112", "v120"}
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            if not modern:
                robot.set_normal_mode()
            if robot.enable():
                if not modern:
                    robot.set_normal_mode()
                return
        except Exception as exc:
            last_error = exc
        time.sleep(0.05)
    raise RuntimeError(f"timed out enabling Nero with firmware driver {firmware}") from last_error

def connect_robot(config: RealArmConfig, logger: logging.Logger):
    ensure_socketcan_ready(config)
    firmware = detect_firmware(config, logger) if config.firmware == "auto" else config.firmware
    if firmware not in _FIRMWARE_NAMES:
        raise ValueError(f"unsupported Nero firmware: {firmware}")
    if config.firmware != "auto":
        logger.info("using configured Nero firmware driver=%s", firmware)
    robot = create_robot(config, firmware)
    try:
        install_can_send_timeout(robot, config.can_send_timeout_s)
        install_can_frame_interval(robot, config.can_frame_interval_s)
        enable_normal_push(robot, config.enable_timeout_s, firmware)
        if not wait_for_feedback(robot, config.startup_timeout_s):
            raise RuntimeError(f"timed out waiting for Nero feedback with firmware driver {firmware}")
        if hasattr(robot, "set_joint_limits_enabled"):
            robot.set_joint_limits_enabled(config.sdk_joint_limits_enabled)
        if hasattr(robot, "set_speed_percent"):
            robot.set_speed_percent(config.speed_percent)
        return robot, firmware
    except Exception:
        robot.disconnect()
        raise

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
        robot_factory: Callable[[RealArmConfig, logging.Logger], tuple[object, str]] | None = None,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError(f"unsupported real arm side: {side}")
        self.side = side
        self.config = real_config
        self.motion_config = motion_config
        self.logger = logger
        self.robot, self.firmware = robot_factory(real_config, logger) if robot_factory else self._connect_robot()
        try:
            self.q_feedback, self.motor_timestamps, _ = read_motor_feedback(
                self.robot, real_config.startup_timeout_s
            )
        except Exception:
            self.robot.disconnect()
            raise
        self.motor_updated_at = np.full(7, time.monotonic())
        self.feedback_warning = False
        self.last_sent_q = self.q_feedback.copy()
        self.js_active = False
        self.next_diagnostics_at = 0.0
        self.diagnostic_warnings: set[str] = set()
    def _connect_robot(self):
        robot, firmware = connect_robot(self.config, self.logger)
        self.logger.info(
            "connected Nero real arm side=%s channel=%s firmware=%s",
            self.side, self.config.channel, firmware,
        )
        return robot, firmware
    def _set_servo_mode(self) -> None:
        if hasattr(self.robot, "set_motion_mode"):
            self.robot.set_motion_mode("js")
        if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
            self.robot.set_auto_set_motion_mode_enabled(False)
    def move_j_startup(self, side: str, q: np.ndarray) -> None:
        if side != self.side:
            raise ValueError(f"{side} arm is not connected")
        target = validate_joint_command(q)
        if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
            self.robot.set_auto_set_motion_mode_enabled(False)
        if hasattr(self.robot, "set_motion_mode"):
            self.robot.set_motion_mode("j")
        dt = 1.0 / self.motion_config.command_hz
        for q_cmd in joint_trajectory(self.motion_config, self.q_feedback, target):
            self.robot.move_j([float(v) for v in q_cmd])
            self.last_sent_q = q_cmd.copy()
            time.sleep(dt)
        self._wait_for_motor_feedback()
    def prepare_teleop(self, side: str) -> np.ndarray:
        if side != self.side:
            raise ValueError(f"{side} arm is not connected")
        hold_q = self._wait_for_motor_feedback()
        # Nero's assembled joint-angle cache can stall, so it is diagnostic-only here.
        aggregate = read_joint_feedback_optional(self.robot)
        delta = None if aggregate is None else float(np.max(np.abs(hold_q - aggregate)))
        self.logger.info(
            "servo feedback side=%s motor_q=%s aggregate_q=%s aggregate_delta_rad=%s",
            self.side, np.array2string(hold_q, precision=6),
            None if aggregate is None else np.array2string(aggregate, precision=6), delta,
        )
        self._set_servo_mode()
        dt = 1.0 / self.motion_config.command_hz
        cycles = max(1, int(np.ceil(self.config.servo_handoff_s / dt)))
        for _ in range(cycles):
            self.robot.move_js([float(v) for v in hold_q])
            self.last_sent_q = hold_q.copy()
            time.sleep(dt)
            self.refresh_feedback()
        self.js_active = True
        self.logger.info(
            "servo handoff complete side=%s hold_q=%s",
            self.side,
            np.array2string(hold_q, precision=6),
        )
        return hold_q
    def current_joints(self, side: str, wait: bool = False) -> np.ndarray:
        if side != self.side:
            raise ValueError(f"{side} arm is not connected")
        if wait:
            self._wait_for_motor_feedback()
        return self.q_feedback.copy()
    def _wait_for_motor_feedback(self) -> np.ndarray:
        q, self.motor_timestamps, _ = read_motor_feedback(self.robot, self.config.startup_timeout_s)
        self.motor_updated_at[:] = time.monotonic()
        self.q_feedback = q
        self.feedback_warning = False
        return q.copy()
    def refresh_feedback(self) -> None:
        feedback = read_motor_feedback_optional(self.robot)
        if feedback is None:
            self._warn_feedback("missing or invalid motor packet")
            return
        q, timestamps, hz = feedback
        now = time.monotonic()
        self.motor_updated_at[(timestamps != self.motor_timestamps) | (hz > 0.0)] = now
        self.motor_timestamps = timestamps
        if np.any(now - self.motor_updated_at > self.config.feedback_stale_s):
            self._warn_feedback("motor packet stream is stale")
            return
        if self.feedback_warning:
            self.logger.info("motor feedback recovered side=%s", self.side)
        self.feedback_warning = False
        self.q_feedback = q
    def _warn_feedback(self, reason: str) -> None:
        if not self.feedback_warning:
            self.logger.warning("motor feedback unavailable side=%s: %s; continuing", self.side, reason)
            self.feedback_warning = True
    def update(self, left_q: np.ndarray | None, right_q: np.ndarray | None, frames: CoordinateFrames, status: str, dt: float) -> None:
        self.refresh_feedback()
        q = left_q if self.side == "left" else right_q
        if q is None:
            if not self.js_active:
                return
            q = self.last_sent_q
        q = validate_joint_command(q)
        if not self.js_active:
            self._set_servo_mode()
        self.robot.move_js([float(v) for v in q])
        self.js_active = True
        self.last_sent_q = q.copy()
    def _diagnostic_message(self, source: str, index: int | None, fields: tuple[str, ...]):
        try:
            getter = getattr(self.robot, source)
            packet = getter() if index is None else getter(index)
            if packet is None or packet.msg is None:
                return None
            return {field: getattr(packet.msg, field, None) for field in fields}
        except Exception as exc:
            key = f"{source}:{index}"
            if key not in self.diagnostic_warnings:
                self.logger.warning("diagnostic feedback unavailable source=%s joint=%s error=%s", source, index, exc)
                self.diagnostic_warnings.add(key)
            return None
    def diagnostics(self) -> dict[str, object] | None:
        now = time.monotonic()
        if self.config.diagnostics_hz <= 0.0 or now < self.next_diagnostics_at:
            return None
        self.next_diagnostics_at = now + 1.0 / self.config.diagnostics_hz
        motors = [
            self._diagnostic_message("get_motor_states", joint, ("position", "velocity", "current", "torque"))
            for joint in range(1, 8)
        ]
        drivers = [
            self._diagnostic_message("get_driver_states", joint, ("vol", "foc_temp", "motor_temp", "bus_current", "foc_status"))
            for joint in range(1, 8)
        ]
        arm = self._diagnostic_message(
            "get_arm_status", None, ("ctrl_mode", "arm_status", "mode_feedback", "motion_status", "err_status")
        )
        return {"arm": arm, "motors": motors, "drivers": drivers}
    def close(self) -> None:
        try:
            try:
                if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
                    self.robot.set_auto_set_motion_mode_enabled(True)
                if self.firmware not in {"v112", "v120"} and hasattr(self.robot, "set_normal_mode"):
                    self.robot.set_normal_mode()
            except Exception:
                pass
        finally:
            self.robot.disconnect()
