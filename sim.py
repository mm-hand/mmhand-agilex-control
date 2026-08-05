from __future__ import annotations

import logging
import time

import numpy as np
from spatialmath import SE3
from spatialgeometry import Axes

from config import MotionConfig, SimConfig
from frames import ArmBases, CoordinateFrames
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
