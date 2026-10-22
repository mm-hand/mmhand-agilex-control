from __future__ import annotations
import json
import logging
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import MethodType
from typing import Callable
import numpy as np
from config import MotionConfig, RealArmConfig
from frames import CoordinateFrames
from joint_feedback import JointFeedbackSample, read_feedback_sample
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
        text = version.strip().lower().removeprefix("v")
        parts = text.split(".")
        if len(parts) == 2:
            parsed = tuple(int(part) for part in parts)
        elif len(parts) == 3 and all(len(part) == 1 for part in parts):
            parsed = (int(parts[0]), int(parts[1]) * 10 + int(parts[2]))
        else:
            raise ValueError
    except (AttributeError, ValueError) as exc:
        raise RuntimeError(f"invalid Nero firmware version: {version!r}") from exc
    if parsed >= (1, 20):
        return "v120"
    if parsed >= (1, 12):
        return "v112"
    if parsed >= (1, 11):
        return "v111"
    return "default"

def _socketcan_state(config: RealArmConfig) -> str | None:
    if config.interface != "socketcan":
        return None
    try:
        result = subprocess.run(
            ["ip", "-details", "-json", "link", "show", config.channel],
            check=True,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
        links = json.loads(result.stdout)
        return str(links[0]["linkinfo"]["info_data"]["state"]).upper()
    except (IndexError, KeyError, OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"failed to inspect SocketCAN interface {config.channel!r}") from exc

_CAN_FAULT_STATES = frozenset({"ERROR-PASSIVE", "BUS-OFF", "STOPPED"})

def _raise_for_can_fault(
    config: RealArmConfig,
    fault_since: float | None = None,
    grace_s: float = 0.0,
) -> float | None:
    """Raise only after the interface has been faulted for ``grace_s`` seconds.

    A freshly powered Nero arm does not acknowledge CAN frames until its
    controller has booted, which drives the host interface into
    ERROR-PASSIVE for a few seconds.  Treating that as fatal aborts the
    connection even though a retry moments later succeeds, so callers pass the
    previous return value back in to measure how long the fault has lasted.
    """
    state = _socketcan_state(config)
    if state not in _CAN_FAULT_STATES:
        return None
    now = time.monotonic()
    if fault_since is None:
        fault_since = now
    if now - fault_since < grace_s:
        return fault_since
    raise RuntimeError(
        f"SocketCAN interface {config.channel!r} entered {state}; stop the arm program, "
        "reset the CAN interface, and verify bitrate, H/L wiring, power, and passive RX traffic"
    )

def _log_sdk_identity(config: RealArmConfig, logger: logging.Logger) -> None:
    _sdk(config)
    import pyAgxArm
    logger.info(
        "using pyAgxArm sdk_version=%s path=%s",
        getattr(pyAgxArm, "__version__", "unknown"),
        Path(pyAgxArm.__file__).resolve(),
    )

def ensure_socketcan_ready(config: RealArmConfig, logger: logging.Logger | None = None) -> None:
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
    # A fault left behind by a previous attempt clears itself once frames are
    # acknowledged again, so report it and let the connect path retry instead of
    # refusing to start before any traffic has been sent.
    state = _socketcan_state(config)
    if state in _CAN_FAULT_STATES and logger is not None:
        logger.warning(
            "SocketCAN interface %r is %s; retrying after enabling CAN push",
            config.channel, state,
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

def _joint_feedback_timestamp(robot) -> float | None:
    try:
        packet = robot.get_joint_angles()
    except Exception:
        return None
    if packet is None:
        return None
    try:
        return float(packet.timestamp)
    except (TypeError, ValueError):
        return None

_push_command_lock = threading.Lock()
_push_command_at = 0.0

def _send_push_command(set_normal_mode) -> None:
    """Send a CAN push request while keeping the arm-side spacing.

    The controller disables push again when two mode-configurations arrive less
    than ~100 ms apart, and firmware detection plus connect each issue one, so
    the gap is enforced across the whole process rather than per driver object.
    """
    global _push_command_at
    with _push_command_lock:
        wait = _push_command_at + 0.2 - time.monotonic()
        if wait > 0.0:
            time.sleep(wait)
        set_normal_mode()
        _push_command_at = time.monotonic()

def _stream_stable(robot, window_s: float, min_updates: int = 5) -> bool:
    """True when feedback keeps advancing for the whole window.

    Checking a single timestamp advance is not enough: the controller sometimes
    accepts a push request and stops streaming again a few tens of milliseconds
    later, so the stream must still be alive at the end of the window.
    """
    deadline = time.monotonic() + window_s
    seen: set[float] = set()
    previous = None
    last_change = None
    while time.monotonic() < deadline:
        timestamp = _joint_feedback_timestamp(robot)
        if timestamp is not None:
            if previous is not None and timestamp != previous:
                last_change = time.monotonic()
            seen.add(timestamp)
            previous = timestamp
        time.sleep(0.02)
    if last_change is None or len(seen) < min_updates:
        return False
    return time.monotonic() - last_change <= 0.1

def enable_can_push(robot, settle_s: float = 0.2, attempts: int = 3) -> bool:
    """Turn on active CAN reporting and confirm that fresh frames arrive.

    Nero firmware at or below 1.11 stops streaming feedback until
    ``set_normal_mode()`` enables CAN push (the vendor's own "no data from
    candump" troubleshooting).  Two ``set_normal_mode()`` calls issued back to
    back leave the controller with push *disabled*, so calls are spaced and the
    result is verified against the feedback timestamp instead of assumed.
    """
    set_normal_mode = getattr(robot, "set_normal_mode", None)
    if set_normal_mode is None:
        return True
    for _ in range(attempts):
        _send_push_command(set_normal_mode)
        if _stream_stable(robot, window_s=settle_s):
            return True
    return False

def detect_firmware(config: RealArmConfig, logger: logging.Logger) -> str:
    robot = create_robot(config, "default")
    deadline = time.monotonic() + config.enable_timeout_s
    next_enable_at = time.monotonic()
    last_error: Exception | None = None
    fault_since: float | None = None
    try:
        install_can_send_timeout(robot, config.can_send_timeout_s)
        while time.monotonic() < deadline:
            try:
                firmware = robot.get_firmware(timeout=0.3, min_interval=1.0)
            except Exception as exc:
                last_error = exc
            else:
                if firmware:
                    software = str(firmware.get("software_version", "")).strip()
                    selected = _firmware_name(software)
                    logger.info("detected Nero arm_firmware=%s driver=%s", software, selected)
                    return selected
            fault_since = _raise_for_can_fault(config, fault_since, config.can_fault_grace_s)
            now = time.monotonic()
            if now >= next_enable_at:
                try:
                    enable_can_push(robot)
                    robot.enable()
                except Exception as exc:
                    last_error = exc
                next_enable_at = now + 1.0
                fault_since = _raise_for_can_fault(config, fault_since, config.can_fault_grace_s)
            time.sleep(0.05)
    finally:
        robot.disconnect()
    message = (
        "timed out detecting Nero firmware; the arm sends no feedback. Check that the arm "
        "finished booting, that CAN push is enabled (firmware <= 1.11 needs set_normal_mode), "
        "and that bitrate, H/L wiring and 24 V power are correct; or set "
        "RealArmConfig.firmware to default/v111/v112/v120 explicitly"
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

def read_joint_feedback(robot, timeout_s: float) -> JointFeedbackSample:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        feedback = read_feedback_sample(robot)
        if feedback is not None:
            return feedback
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for joint-angle and motor-state feedback")

def read_current_joint_angles(robot, timeout_s: float) -> np.ndarray:
    return read_joint_feedback(robot, timeout_s)[0]

def wait_for_feedback(robot, timeout_s: float) -> bool:
    """Wait until the arm is actively streaming, not merely cached.

    The SDK keeps returning the last parsed packet, so a silent arm still looks
    "available"; require the joint timestamp to advance instead.
    """
    deadline = time.monotonic() + timeout_s
    first_timestamp: float | None = None
    while time.monotonic() < deadline:
        sample = read_feedback_sample(robot)
        if sample is not None:
            timestamp = float(sample[1])
            if first_timestamp is None:
                first_timestamp = timestamp
            elif timestamp != first_timestamp:
                return True
        time.sleep(0.02)
    return False

def enable_normal_push(robot, timeout_s: float, firmware: str) -> None:
    modern = firmware in {"v112", "v120"}
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            enabled = robot.enable()
        except Exception as exc:
            last_error = exc
            enabled = False
        if enabled:
            # Push is enabled exactly once and only after enable(): a second
            # set_normal_mode() immediately after the first turns it back off.
            if modern or enable_can_push(robot):
                return
            last_error = RuntimeError("Nero CAN push did not start")
        time.sleep(0.05)
    raise RuntimeError(f"timed out enabling Nero with firmware driver {firmware}") from last_error

def connect_robot(config: RealArmConfig, logger: logging.Logger):
    _log_sdk_identity(config, logger)
    ensure_socketcan_ready(config, logger)
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
            # The controller drops CAN push again on some mode-message races, so
            # re-request it once instead of failing straight away.
            logger.warning("no Nero feedback after enabling; re-requesting CAN push")
            if enable_can_push(robot) and wait_for_feedback(robot, config.startup_timeout_s):
                logger.info("Nero feedback recovered side=%s", config.channel)
            else:
                _raise_for_can_fault(config)
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
        self.feedback_lock = threading.Lock()
        self.robot, self.firmware = robot_factory(real_config, logger) if robot_factory else self._connect_robot()
        try:
            feedback = read_joint_feedback(self.robot, real_config.startup_timeout_s)
        except Exception:
            self.robot.disconnect()
            raise
        self.q_feedback, self.joint_timestamp, _, self.motor_timestamps = feedback
        self.latest_feedback_sample = feedback
        now = time.monotonic()
        self.joint_updated_at = now
        self.motor_updated_at = np.full(7, now)
        self.feedback_warning = False
        self._push_rearm_at = 0.0
        self.last_sent_q = self.q_feedback.copy()
        self.js_active = False
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
    def move_j_startup(self, side: str, q: np.ndarray, stop_event=None) -> bool:
        if side != self.side:
            raise ValueError(f"{side} arm is not connected")
        target = validate_joint_command(q)
        if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
            self.robot.set_auto_set_motion_mode_enabled(False)
        if hasattr(self.robot, "set_motion_mode"):
            self.robot.set_motion_mode("j")
        dt = 1.0 / self.motion_config.command_hz
        for q_cmd in joint_trajectory(self.motion_config, self.q_feedback, target):
            if stop_event is not None and stop_event.is_set():
                self.logger.warning("startup move aborted side=%s", self.side)
                return False
            self.robot.move_j([float(v) for v in q_cmd])
            self.last_sent_q = q_cmd.copy()
            time.sleep(dt)
        self._latest_feedback()
        return True

    def prepare_teleop(self, side: str, stop_event=None) -> np.ndarray | None:
        if side != self.side:
            raise ValueError(f"{side} arm is not connected")
        fresh = self.refresh_feedback()
        hold_q = self.q_feedback.copy() if fresh else self.last_sent_q.copy()
        self._set_servo_mode()
        dt = 1.0 / self.motion_config.command_hz
        cycles = max(1, int(np.ceil(self.config.servo_handoff_s / dt)))
        for _ in range(cycles):
            if stop_event is not None and stop_event.is_set():
                self.logger.warning("servo handoff aborted side=%s", self.side)
                return None
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
            self._latest_feedback()
        with self.feedback_lock:
            return self.q_feedback.copy()
    def _latest_feedback(self) -> np.ndarray:
        self.refresh_feedback()
        return self.current_joints(self.side)
    def feedback_sample(self) -> JointFeedbackSample:
        with self.feedback_lock:
            joint_q, joint_timestamp, motor_q, motor_timestamps = self.latest_feedback_sample
            return joint_q.copy(), joint_timestamp, motor_q.copy(), motor_timestamps.copy()
    def refresh_feedback(self) -> bool:
        feedback = read_feedback_sample(self.robot)
        if feedback is None:
            self._warn_feedback("missing or invalid joint-angle or motor-state packet")
            return False
        with self.feedback_lock:
            joint_q, joint_timestamp, _, motor_timestamps = feedback
            now = time.monotonic()
            if joint_timestamp != self.joint_timestamp:
                self.joint_updated_at = now
            self.motor_updated_at[motor_timestamps != self.motor_timestamps] = now
            self.joint_timestamp = joint_timestamp
            self.motor_timestamps = motor_timestamps
            self.latest_feedback_sample = feedback
            if (
                now - self.joint_updated_at > self.config.feedback_stale_s
                or np.any(now - self.motor_updated_at > self.config.feedback_stale_s)
            ):
                self._warn_feedback("joint-angle or motor-state packet stream is stale")
                self._rearm_can_push()
                return False
            if self.feedback_warning:
                self.logger.info("joint feedback recovered side=%s", self.side)
            self.feedback_warning = False
            self.q_feedback = joint_q
            return True
    def _rearm_can_push(self) -> None:
        """Re-request CAN push when the arm goes silent, at most once a second."""
        now = time.monotonic()
        if now - self._push_rearm_at < 1.0:
            return
        self._push_rearm_at = now
        set_normal_mode = getattr(self.robot, "set_normal_mode", None)
        if set_normal_mode is None:
            return
        try:
            _send_push_command(set_normal_mode)
            self.logger.warning("re-requested CAN push side=%s", self.side)
        except Exception:
            pass
    def _warn_feedback(self, reason: str) -> None:
        if not self.feedback_warning:
            self.logger.warning("joint feedback unavailable side=%s: %s; continuing", self.side, reason)
            self.feedback_warning = True
    def update(self, left_q: np.ndarray | None, right_q: np.ndarray | None, frames: CoordinateFrames, status: str, dt: float) -> None:
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
    def close(self) -> None:
        try:
            try:
                if hasattr(self.robot, "set_auto_set_motion_mode_enabled"):
                    self.robot.set_auto_set_motion_mode_enabled(True)
                if self.firmware not in {"v112", "v120"} and hasattr(self.robot, "set_normal_mode"):
                    _send_push_command(self.robot.set_normal_mode)
            except Exception:
                pass
        finally:
            self.robot.disconnect()
