from __future__ import annotations

import csv
import json
import logging
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from spatialmath import SE3
from spatialgeometry import Axes

from config import MotionConfig, SimConfig
from frames import ArmBases, ControllerPose, CoordinateFrames, arm_base_in_horizon, build_coordinate_frames
from motion import plan_joint_trajectory

# Swift world is Z-up; input horizon is Y-up.
# Swift_X = hX, Swift_Y = -hZ, Swift_Z = hY.
_SWIFT_FROM_HORIZON = np.eye(4)
_SWIFT_FROM_HORIZON[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])


def swift_se3(transform_horizon: np.ndarray, z_offset_m: float = 0.0) -> SE3:
    transform = _SWIFT_FROM_HORIZON @ transform_horizon
    transform[:3, 3] += np.array([0.0, 0.0, z_offset_m])
    return SE3.Rt(transform[:3, :3], transform[:3, 3])


def _patch_swift_reconnect_leak():
    import asyncio
    import json
    import socketserver
    import webbrowser
    from queue import Empty

    import swift.SwiftRoute as sr
    from swift.Swift import Swift

    socketserver.TCPServer.allow_reuse_address = True

    state = {
        "handshake_done": False,
        "connected": False,
        "active_ws": None,
        "epoch": 0,
        "was_connected": True,
    }

    _orig_open = webbrowser.open_new_tab

    def _open_and_print(url, *args, **kwargs):
        print(f"Swift URL: {url}", flush=True)
        return _orig_open(url, *args, **kwargs)

    webbrowser.open_new_tab = _open_and_print

    async def serve(self, websocket, path) -> None:
        my_epoch = state["epoch"] + 1
        state["epoch"] = my_epoch
        old_ws = state["active_ws"]
        if old_ws is not None and old_ws is not websocket:
            try:
                await old_ws.close()
            except Exception:
                pass
        state["active_ws"] = websocket
        await self.register(websocket)
        await websocket.recv()
        if not state["handshake_done"]:
            self.inq.put("Connected")
            state["handshake_done"] = True
        else:
            while not self.outq.empty():
                try:
                    self.outq.get_nowait()
                except Exception:
                    break
            while not self.inq.empty():
                try:
                    self.inq.get_nowait()
                except Exception:
                    break
        state["connected"] = True
        try:
            while self.run() and state["epoch"] == my_epoch:
                if websocket.closed:
                    break
                try:
                    message = self.outq.get_nowait()
                except Empty:
                    await asyncio.sleep(0.02)
                    continue
                await websocket.send(json.dumps(message[1]))
                await self.expect_message(websocket, message[0])
        finally:
            if state["epoch"] == my_epoch:
                state["connected"] = False
                state["active_ws"] = None

    def _send_socket(self, code, data=None, expected=True):
        if not state["connected"]:
            return "{}" if code == "shape_poses" else "0"
        self.outq.put([expected, [code, data]])
        if expected:
            try:
                return self.inq.get(timeout=1.0)
            except Empty:
                state["connected"] = False
                return "{}" if code == "shape_poses" else "0"
        return "0"

    def browser_reconnected() -> bool:
        c = state["connected"]
        changed = c and not state["was_connected"]
        state["was_connected"] = c
        return changed

    def drain_queues(env) -> None:
        while not env.outq.empty():
            try:
                env.outq.get_nowait()
            except Exception:
                break
        while not env.inq.empty():
            try:
                env.inq.get_nowait()
            except Exception:
                break

    def is_connected() -> bool:
        return state["connected"]

    sr.SwiftSocket.serve = serve
    Swift._send_socket = _send_socket
    return browser_reconnected, drain_queues, is_connected


def _reemit_scene(env, items, is_connected) -> None:
    import roboticstoolbox as rtb
    from spatialgeometry import Shape
    from swift.SwiftElement import SwiftElement

    for ob in items:
        if not is_connected():
            return
        if isinstance(ob, rtb.Robot):
            ob._update_link_tf()
            ob._propogate_scene_tree()
            robob = ob._to_dict()
            shape_id = env._send_socket("shape", robob)
            for _ in range(100):
                if not is_connected():
                    return
                if int(env._send_socket("shape_mounted", [shape_id, len(robob)])):
                    break
                time.sleep(0.1)
        elif isinstance(ob, Shape):
            ob._propogate_scene_tree()
            shape_id = int(env._send_socket("shape", [ob.to_dict()]))
            for _ in range(100):
                if not is_connected():
                    return
                if int(env._send_socket("shape_mounted", [shape_id, 1])):
                    break
                time.sleep(0.1)
        elif isinstance(ob, SwiftElement):
            env._send_socket("element", ob.to_dict())


class SwiftSimBackend:
    def __init__(self, left_robot, right_robot, bases: ArmBases, sim_config: SimConfig, motion_config: MotionConfig, logger: logging.Logger) -> None:
        from swift import Swift
        from swift.SwiftElement import Label

        self.logger = logger
        self.motion_config = motion_config
        self.z_offset_m = sim_config.visual_z_offset_m
        self.browser_reconnected, self.drain_queues, self.is_connected = _patch_swift_reconnect_leak()

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
        robot = self.left_robot if side == "left" else self.right_robot if side == "right" else None
        if robot is None:
            raise ValueError(f"unsupported arm side: {side}")
        dt = 1.0 / self.motion_config.command_hz
        self.status_label.desc = f"moving {side} arm to startup pose"
        for point in plan_joint_trajectory(self.motion_config, robot.q, q):
            robot.q = point
            self.env.step(dt)
            time.sleep(dt)

    def update(
        self,
        left_q: np.ndarray | None,
        right_q: np.ndarray | None,
        frames: CoordinateFrames,
        status: str,
        dt: float,
    ) -> None:
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

        if self.browser_reconnected():
            self.drain_queues(self.env)
            _reemit_scene(self.env, self.items, self.is_connected)

        self.status_label.desc = status
        self.env.step(dt)

    def close(self) -> None:
        try:
            self.env.close()
        except Exception:
            pass


def _comparison_fk(robot, base: np.ndarray, q: np.ndarray) -> np.ndarray:
    return base @ robot.fkine(q, end="revo2_flange").A


def _comparison_error(real: np.ndarray, sim: np.ndarray) -> tuple[float, float]:
    position = float(np.linalg.norm(real[:3, 3] - sim[:3, 3]))
    cosine = np.clip((np.trace(real[:3, :3].T @ sim[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return position, float(np.degrees(np.arccos(cosine)))


def replay_comparison(samples: list[tuple[float, ControllerPose, np.ndarray, np.ndarray]], config, logger) -> None:
    if not samples:
        logger.info("comparison skipped: no valid tracked samples")
        return

    from ik import NeroIK

    left, right = NeroIK(config.ik), NeroIK(config.ik)
    bases = arm_base_in_horizon(config.frames)
    rows = []
    for elapsed, pico, command_q, real_q in samples:
        real = _comparison_fk(left.robot, bases.left, real_q)
        sim = _comparison_fk(left.robot, bases.left, command_q)
        rows.append((elapsed, pico, command_q, real_q, real, sim, *_comparison_error(real, sim)))

    directory = Path(config.log.comparison_directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{datetime.now():%Y%m%d_%H%M%S}_left.csv"
    compact = lambda value: json.dumps(np.asarray(value, dtype=float).tolist(), separators=(",", ":"))
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("elapsed_s", "pico", "command_q", "real_q", "real_flange", "sim_flange", "position_error_m", "rotation_error_deg"))
        for elapsed, pico, command_q, real_q, real, sim, position, rotation in rows:
            writer.writerow((elapsed, compact((*pico.xyz_m, *pico.quat_xyzw)), compact(command_q), compact(real_q), compact(real), compact(sim), position, rotation))

    position = np.asarray([row[-2] for row in rows])
    rotation = np.asarray([row[-1] for row in rows])
    logger.info("comparison samples=%d position_rms=%.2fmm max=%.2fmm rotation_rms=%.2fdeg max=%.2fdeg csv=%s", len(rows), 1000.0 * np.sqrt(np.mean(position**2)), 1000.0 * np.max(position), np.sqrt(np.mean(rotation**2)), np.max(rotation), path)

    backend = None
    try:
        left.robot.q = rows[0][2].copy()
        backend = SwiftSimBackend(left.robot, right.robot, bases, config.sim, config.motion, logger)
        previous = rows[0][0]
        for elapsed, pico, command_q, _real_q, real, _sim, position, rotation in rows:
            tick = time.monotonic()
            dt = max(elapsed - previous, 1.0 / config.motion.command_hz)
            frames = build_coordinate_frames(pico, None, config.frames)
            if frames.left is None:
                continue
            frames = CoordinateFrames(frames.left._replace(transform_horizon_to_flange=real), None)
            backend.update(command_q, None, frames, f"real vs sim: {position * 1000.0:.1f} mm | {rotation:.1f} deg", dt)
            previous = elapsed
            time.sleep(max(0.0, dt - (time.monotonic() - tick)))
        input("replay complete; press Enter to close Swift...")
    except KeyboardInterrupt:
        pass
    finally:
        if backend is not None:
            backend.close()
