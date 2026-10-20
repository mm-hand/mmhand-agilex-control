from __future__ import annotations

import logging
import queue
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace

import numpy as np

from config import ControlConfig
from controller_state import ArmStatus, ControlCommand, ControlSnapshot, ControllerSnapshot, SnapshotStore
from filters import JointTargetFilter, PoseFilter, PoseInputLimiter
from frames import CoordinateFrames, ControllerPose, arm_base_in_horizon, build_coordinate_frames
from hardware import RealArmBackend
from ik import NeroIK
from motion import ArmMotionPlanner, joint_trajectory
from ros_output import RosOutput

SIDES = ("left", "right")


class _StopSignal:
    def __init__(self, *events: threading.Event) -> None:
        self._events = events

    def is_set(self) -> bool:
        return any(event.is_set() for event in self._events)


def _startup_pose(config: ControlConfig, side: str) -> np.ndarray:
    return np.asarray(getattr(config.startup, f"{side}_pose_rad"), dtype=float).reshape(7)


def _controller_snapshot(pose: ControllerPose | None, buttons: int = 0) -> ControllerSnapshot | None:
    if pose is None:
        return None
    return ControllerSnapshot(
        valid=True,
        xyz=tuple(float(value) for value in pose.xyz_m),
        quat=tuple(float(value) for value in pose.quat_xyzw),
        buttons=int(buttons),
    )


class ArmController:
    def __init__(
        self,
        config: ControlConfig,
        logger: logging.Logger,
        pose_source,
        store: SnapshotStore,
        arm_factory=None,
    ) -> None:
        self.config = config
        self.logger = logger
        self.pose_source = pose_source
        self.store = store
        self.commands: queue.Queue[ControlCommand] = queue.Queue()
        self.iks = {side: NeroIK(config.ik) for side in SIDES}
        self.bases = arm_base_in_horizon(config.frames)
        self.sim_q = {side: np.zeros(7, dtype=float) for side in SIDES}
        self.real_backends: dict[str, object] = {}
        self.following: set[str] = set()
        self.arm_status = {side: ArmStatus() for side in SIDES}
        self.real_q: dict[str, np.ndarray | None] = {side: None for side in SIDES}
        self.ros = RosOutput(config.ros_output, logger)
        self.phase = "STARTUP"
        self.error: str | None = None
        self.handoff_id = 0
        self.handoff_sides: tuple[str, ...] = ()
        self._arm_factory = arm_factory or RealArmBackend
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="arm")
        self._connect_futures: dict[str, Future] = {}
        self._align_futures: dict[str, Future] = {}
        self._busy: set[str] = set()
        self._holds: dict[str, np.ndarray] = {}
        self._last_hold: dict[str, np.ndarray] = {side: self.sim_q[side].copy() for side in SIDES}
        self._startup: dict[str, list[np.ndarray]] = {}
        self._startup_index = 0
        self._stop_motion = threading.Event()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="arm-controller", daemon=True)
        self._pose_filters = {}
        self._limiters = {}
        self._target_filters = {}
        self._planners = {}
        store.publish(ControlSnapshot())

    def start(self) -> None:
        self._thread.start()

    def submit(self, command: ControlCommand) -> None:
        self.commands.put(command)

    def stop(self) -> None:
        self._stop_motion.set()
        self._stop_event.set()

    def wait(self, timeout: float | None = None) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        try:
            self._begin_startup()
            period = 1.0 / self.config.motion.command_hz
            while not self._stop_event.is_set():
                started = time.monotonic()
                self._poll_futures()
                self._drain_commands()
                self._tick()
                elapsed = time.monotonic() - started
                self._stop_event.wait(max(0.0, period - elapsed))
        except Exception as exc:
            self._fail("controller loop failed", exc)
        finally:
            self._shutdown()

    def _tick(self) -> None:
        if self.phase == "STARTUP":
            self._step_startup()
            self._hold_following()
        elif self.phase in ("HANDOFF", "ALIGNING"):
            self._hold_following()
        elif self.phase == "TELEOP":
            self._step_teleop()
        elif self.phase in ("STOPPED", "FAULT"):
            self._hold_following()
        self._refresh_feedback()
        self._publish_ros()
        self._publish_snapshot()

    # ------------------------------------------------------------------ startup
    def _begin_startup(self) -> None:
        if not self.config.startup.enable_startup_pose:
            self.sim_q = {side: _startup_pose(self.config, side) for side in SIDES}
            self._finish_startup()
            return
        self._startup = {}
        for side in SIDES:
            target = _startup_pose(self.config, side)
            self._startup[side] = joint_trajectory(self.config.motion, self.sim_q[side], target)
        self._startup_index = 0
        self.phase = "STARTUP"
        for side in self.following:
            self.arm_status[side] = replace(self.arm_status[side], state="READY")
        self.logger.info("simulation startup motion started")

    def _step_startup(self) -> None:
        finished = True
        for side in SIDES:
            points = self._startup[side]
            index = min(self._startup_index, len(points) - 1)
            self.sim_q[side] = points[index].copy()
            if self._startup_index < len(points) - 1:
                finished = False
        self._startup_index += 1
        if finished:
            self._finish_startup()

    def _finish_startup(self) -> None:
        self.logger.info("simulation startup pose reached")
        if self.real_backends:
            self._open_handoff()
        else:
            self._enter_teleop()

    # ------------------------------------------------------------------ handoff
    def _open_handoff(self) -> None:
        sides = tuple(sorted(self.real_backends))
        if not sides:
            self._enter_teleop()
            return
        self.phase = "HANDOFF"
        self.handoff_id += 1
        self.handoff_sides = sides
        self.logger.info("handoff requested sides=%s", ",".join(sides))

    def _reopen_handoff(self) -> None:
        if self.phase == "HANDOFF":
            self._open_handoff()
        elif self.phase == "TELEOP":
            self.logger.info("new arm connected during teleop; returning simulation to startup pose")
            self._begin_startup()

    def _begin_alignment(self, request_id: int) -> None:
        if self.phase != "HANDOFF" or request_id != self.handoff_id or not self.real_backends:
            return
        self._holds = {}
        self._align_futures = {}
        for side in tuple(self.real_backends):
            backend = self.real_backends[side]
            self._busy.add(side)
            self.arm_status[side] = replace(self.arm_status[side], state="ALIGNING", error=None)
            self._align_futures[side] = self._executor.submit(self._align_worker, side, backend)
        self.phase = "ALIGNING"
        self.logger.info("real arm alignment started sides=%s", ",".join(sorted(self.real_backends)))

    def _align_worker(self, side: str, backend) -> np.ndarray:
        target = _startup_pose(self.config, side)
        stop = _StopSignal(self._stop_motion, self._stop_event)
        if not backend.move_j_startup(side, target, stop):
            raise RuntimeError(f"{side} startup move aborted")
        hold = backend.prepare_teleop(side, stop)
        if hold is None:
            raise RuntimeError(f"{side} servo handoff aborted")
        return np.asarray(hold, dtype=float).reshape(7)

    def _poll_align(self) -> None:
        if not self._align_futures:
            return
        if self.phase != "ALIGNING":
            for side in list(self._align_futures):
                future = self._align_futures[side]
                if not future.done():
                    continue
                del self._align_futures[side]
                self._busy.discard(side)
                try:
                    future.result()
                except Exception:
                    pass
                if side not in self.following:
                    backend = self.real_backends.pop(side, None)
                    self.arm_status[side] = replace(self.arm_status[side], state="STOPPED")
                    if backend is not None:
                        self._executor.submit(self._close_worker, backend)
            return
        for side in list(self._align_futures):
            future = self._align_futures[side]
            if not future.done():
                continue
            del self._align_futures[side]
            self._busy.discard(side)
            try:
                self._holds[side] = future.result()
            except Exception as exc:
                if self._stop_motion.is_set():
                    self._disconnect_side(side, "STOPPED")
                else:
                    self._fail(f"failed to align {side} arm", exc)
                return
        if not self._align_futures:
            self._finish_alignment()

    def _finish_alignment(self) -> None:
        starts = {side: self.sim_q[side].copy() for side in SIDES}
        for side, hold in self._holds.items():
            starts[side] = hold
            self.sim_q[side] = hold.copy()
        self.following = set(self._holds)
        self._enter_teleop(starts)

    # ------------------------------------------------------------------ teleop
    def _enter_teleop(self, starts: dict[str, np.ndarray] | None = None) -> None:
        starts = starts or {side: self.sim_q[side].copy() for side in SIDES}
        for side in SIDES:
            self.iks[side].reset_state(starts[side])
        self._pose_filters = {side: PoseFilter(self.config.filters.pose) for side in SIDES}
        self._limiters = {side: PoseInputLimiter(self.config.input_limit) for side in SIDES}
        self._target_filters = {side: JointTargetFilter(self.config.filters.ik_output) for side in SIDES}
        self._planners = {side: ArmMotionPlanner(self.config.motion, starts[side]) for side in SIDES}
        self._last_hold = {side: starts[side].copy() for side in SIDES}
        self.phase = "TELEOP"
        self.handoff_sides = ()
        self.logger.info("teleop started following=%s", ",".join(sorted(self.following)) or "none")

    def _step_teleop(self) -> None:
        now = time.monotonic()
        raw = {"left": self.pose_source.get_left(), "right": self.pose_source.get_right()}
        poses = {}
        for side in SIDES:
            pose = self._pose_filters[side].apply(raw[side], now)
            poses[side] = self._limiters[side].apply(pose, now)
        frames = build_coordinate_frames(poses["left"], poses["right"], self.config.frames)
        commands = {}
        for side in SIDES:
            target = None
            frame = getattr(frames, side)
            if frame is not None:
                target = self.iks[side].solve(frame.transform_arm_base_to_flange)
                target = self._target_filters[side].apply(target, now)
            commands[side] = self._planners[side].step(target, now)
        for side in self.following:
            backend = self.real_backends.get(side)
            if backend is None or side in self._busy:
                continue
            backend.update(
                commands["left"] if side == "left" else None,
                commands["right"] if side == "right" else None,
                frames,
                "",
                1.0 / self.config.motion.command_hz,
            )
        for side in SIDES:
            self.sim_q[side] = commands[side].copy()
            self._last_hold[side] = commands[side].copy()

    def _hold_following(self) -> None:
        for side in self.following:
            backend = self.real_backends.get(side)
            if backend is None or side in self._busy:
                continue
            q = self._last_hold[side]
            backend.update(
                q if side == "left" else None,
                q if side == "right" else None,
                CoordinateFrames(None, None),
                "",
                1.0 / self.config.motion.command_hz,
            )

    # ------------------------------------------------------------------ commands
    def _drain_commands(self) -> None:
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            self._handle_command(command)

    def _handle_command(self, command: ControlCommand) -> None:
        if self.phase == "FAULT":
            return
        if self.phase == "STOPPED" and command.kind not in ("STOP", "DISCONNECT", "RESET"):
            return
        if command.kind == "DISCONNECT" and (command.side in self._busy or self.phase == "ALIGNING"):
            return
        if command.kind == "STOP":
            self._stop_all()
        elif command.kind == "RESET":
            self._reset_stop()
        elif command.kind == "CONNECT":
            self._connect(command.side, command.channel)
        elif command.kind == "DISCONNECT":
            self._disconnect_side(command.side)
        elif command.kind == "CONFIRM":
            self._begin_alignment(command.request_id)
        elif command.kind == "CANCEL":
            self._cancel_handoff()
        elif command.kind == "SET_ROS":
            self._set_ros(command.enabled)

    def _connect(self, side: str, channel: str) -> None:
        if side not in SIDES or side in self.real_backends or side in self._busy:
            return
        if self.phase in ("ALIGNING", "STOPPED", "FAULT"):
            return
        channel = channel or self.config.real.channel
        for other in SIDES:
            if other == side:
                continue
            status = self.arm_status[other]
            if status.state in ("CONNECTING", "READY", "TELEOP", "ALIGNING") and status.channel == channel:
                self.logger.warning("refusing duplicate channel %s", channel)
                self.arm_status[side] = replace(self.arm_status[side], state="ERROR", error="channel already in use")
                return
        self._busy.add(side)
        self.arm_status[side] = replace(self.arm_status[side], state="CONNECTING", channel=channel, error=None)
        self._connect_futures[side] = self._executor.submit(self._connect_worker, side, channel)

    def _connect_worker(self, side: str, channel: str):
        config = replace(self.config.real, channel=channel or self.config.real.channel)
        return self._arm_factory(side, config, self.config.motion, self.logger)

    def _poll_connect(self) -> None:
        for side in list(self._connect_futures):
            future = self._connect_futures[side]
            if not future.done():
                continue
            del self._connect_futures[side]
            self._busy.discard(side)
            try:
                backend = future.result()
            except Exception as exc:
                self.arm_status[side] = replace(self.arm_status[side], state="ERROR", error=str(exc))
                self.logger.warning("failed to connect %s arm: %s", side, exc)
                continue
            if self._stop_motion.is_set() or self._stop_event.is_set() or self.phase in ("STOPPED", "FAULT"):
                self._executor.submit(self._close_worker, backend)
                self.arm_status[side] = replace(self.arm_status[side], state="STOPPED")
                continue
            self.real_backends[side] = backend
            self.arm_status[side] = ArmStatus(
                state="READY",
                channel=getattr(backend, "config", self.config.real).channel,
            )
            self.logger.info("connected %s arm", side)
            self._reopen_handoff()

    def _disconnect_side(self, side: str, state: str = "SIM") -> None:
        backend = self.real_backends.pop(side, None)
        self.following.discard(side)
        self.real_q[side] = None
        self._last_hold[side] = self.sim_q[side].copy()
        self.arm_status[side] = self.arm_status[side].__class__(state=state)
        if backend is not None:
            self._executor.submit(self._close_worker, backend)
        if self.phase == "HANDOFF":
            if self.real_backends:
                self._open_handoff()
            else:
                self._enter_teleop()
        elif self.phase == "ALIGNING" and not self.real_backends and not self._stop_motion.is_set():
            self._enter_teleop()

    def _cancel_handoff(self) -> None:
        if self.phase != "HANDOFF":
            return
        for side in list(self.real_backends):
            self._disconnect_side(side)
        self.logger.info("handoff cancelled; simulation only")
        self._enter_teleop()

    def _stop_all(self) -> None:
        self._stop_motion.set()
        self.phase = "STOPPED"
        self.handoff_sides = ()
        for side in SIDES:
            if side in self.real_backends:
                self.arm_status[side] = replace(self.arm_status[side], state="STOPPED")
        self.logger.warning("STOP requested; holding all arms")

    def _reset_stop(self) -> None:
        if self.phase != "STOPPED":
            return
        if self._busy or self._align_futures or self._connect_futures:
            self.logger.warning("STOP reset deferred while a hardware operation is active")
            return
        self._stop_motion.clear()
        self.error = None
        self.handoff_sides = ()
        for side in self.real_backends:
            self.arm_status[side] = replace(self.arm_status[side], state="READY")
        self.logger.warning("STOP reset; returning simulation to startup")
        self._begin_startup()

    def _set_ros(self, enabled: bool) -> None:
        ok = self.ros.set_enabled(enabled)
        if ok:
            self.pose_source.send_command("set_ros_output", enabled=enabled)
        else:
            self.logger.warning("ROS 2 output unavailable: %s", self.ros.reason)

    # ------------------------------------------------------------------ futures
    def _poll_futures(self) -> None:
        self._poll_connect()
        self._poll_align()

    def _close_worker(self, backend) -> None:
        try:
            backend.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ telemetry
    def _refresh_feedback(self) -> None:
        for side, backend in self.real_backends.items():
            if side in self._busy:
                continue
            try:
                backend.refresh_feedback()
                self.real_q[side] = backend.current_joints(side)
            except Exception as exc:
                self.arm_status[side] = replace(self.arm_status[side], error=str(exc))

    def _publish_ros(self) -> None:
        if not self.ros.enabled:
            return
        for side in SIDES:
            current = self.real_q[side] if side in self.following else self.sim_q[side]
            target = self.sim_q[side]
            if current is None:
                current = target
            current_pose = self.bases.left @ self.iks[side].fk_flange(current) if side == "left" else self.bases.right @ self.iks[side].fk_flange(current)
            target_pose = self.bases.left @ self.iks[side].fk_flange(target) if side == "left" else self.bases.right @ self.iks[side].fk_flange(target)
            self.ros.publish(side, current, target, current_pose, target_pose)

    def _publish_snapshot(self) -> None:
        buttons = self.pose_source.buttons()
        controllers = {
            "left": _controller_snapshot(self.pose_source.get_left(), buttons[0]),
            "right": _controller_snapshot(self.pose_source.get_right(), buttons[1]),
        }
        self.store.publish(
            ControlSnapshot(
                phase=self.phase,
                pico=self.pose_source.status(),
                controllers=controllers,
                sim_q={side: tuple(self.sim_q[side]) for side in SIDES},
                real_q={side: (tuple(self.real_q[side]) if self.real_q[side] is not None else None) for side in SIDES},
                arm_status=dict(self.arm_status),
                ros={
                    "available": self.ros.available,
                    "enabled": self.ros.enabled,
                    "reason": self.ros.reason,
                },
                handoff_id=self.handoff_id,
                handoff_sides=self.handoff_sides,
                error=self.error,
                can_reset=self.phase == "STOPPED"
                and not self._busy
                and not self._align_futures
                and not self._connect_futures,
            )
        )

    # ------------------------------------------------------------------ failure
    def _fail(self, message: str, exc: Exception) -> None:
        self.logger.exception("%s: %s", message, exc)
        self.error = f"{message}: {exc}"
        self.phase = "FAULT"
        self._stop_motion.set()
        self.handoff_sides = ()
        self._publish_snapshot()
        for side, backend in list(self.real_backends.items()):
            if side not in self.following:
                self.real_backends.pop(side, None)
                self.arm_status[side] = replace(self.arm_status[side], state="ERROR", error=str(exc))
                self._close_worker(backend)

    def _shutdown(self) -> None:
        self._stop_event.set()
        self._executor.shutdown(wait=False)
        for backend in list(self.real_backends.values()):
            self._close_worker(backend)
        self.real_backends.clear()
        self.ros.close()
