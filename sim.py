from __future__ import annotations

import time

import numpy as np
from spatialmath import SE3
from spatialgeometry import Axes

from config import FrameTransformConfig, IkConfig, RosConfig, SimConfig
from frames import arm_base_in_horizon, build_coordinate_frames
from ik import NeroIK
from ros_io import PicoPoseSubscriber

# Swift world is Z-up; PICO horizon is Y-up.
# Swift_X = hX, Swift_Y = -hZ, Swift_Z = hY.
_SWIFT_FROM_HORIZON = np.eye(4)
_SWIFT_FROM_HORIZON[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])


def _swift_se3(transform_horizon: np.ndarray) -> SE3:
    transform = _SWIFT_FROM_HORIZON @ transform_horizon
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
                expected = message[0]
                msg = message[1]
                await websocket.send(json.dumps(msg))
                await self.expect_message(websocket, expected)
        finally:
            if state["epoch"] == my_epoch:
                state["connected"] = False
                state["active_ws"] = None

    def _send_socket(self, code, data=None, expected=True):
        if not state["connected"]:
            return "{}" if code == "shape_poses" else "0"
        msg = [expected, [code, data]]
        self.outq.put(msg)
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


def run_sim(
    ros_cfg: RosConfig,
    frame_cfg: FrameTransformConfig,
    ik_cfg: IkConfig,
    sim_cfg: SimConfig,
) -> int:
    from swift import Swift
    from swift.SwiftElement import Label

    browser_reconnected, drain_queues, is_connected = _patch_swift_reconnect_leak()

    sub = PicoPoseSubscriber(ros_cfg)
    left_ik = NeroIK(ik_cfg)
    right_ik = NeroIK(ik_cfg)

    bases = arm_base_in_horizon(frame_cfg)
    left_ik.robot.base = _swift_se3(bases.left)
    right_ik.robot.base = _swift_se3(bases.right)

    left_controller, right_controller, left_flange, right_flange, left_base, right_base = (
        Axes(0.05) for _ in range(6)
    )
    left_base.T = _swift_se3(bases.left).A
    right_base.T = _swift_se3(bases.right).A

    status_label = Label("PICO: starting...")

    items = (
        left_ik.robot,
        right_ik.robot,
        left_controller,
        right_controller,
        left_flange,
        right_flange,
        left_base,
        right_base,
        status_label,
    )

    env = Swift()
    env.launch(realtime=True)
    for item in items:
        env.add(item)
    print("swift ready", flush=True)

    dt = 1.0 / sim_cfg.step_hz
    n = 0
    last_status = ""
    pico_stale_s = ros_cfg.pico_disconnect_s
    # IK hold print rate limit: at most once per second per arm.
    last_hold_print = [0.0, 0.0]  # [left, right]
    try:
        while True:
            sub.spin()
            frames = build_coordinate_frames(sub.get_left(), sub.get_right(), frame_cfg)

            if frames.left is not None:
                q = left_ik.solve(frames.left.transform_arm_base_to_flange)
                if q is not None:
                    left_ik.robot.q = q
                else:
                    now = time.monotonic()
                    if now - last_hold_print[0] > 1.0:
                        print("[IK] L hold", flush=True)
                        last_hold_print[0] = now
                left_controller.T = _swift_se3(frames.left.transform_horizon_to_controller).A
                left_flange.T = _swift_se3(frames.left.transform_horizon_to_flange).A

            if frames.right is not None:
                q = right_ik.solve(frames.right.transform_arm_base_to_flange)
                if q is not None:
                    right_ik.robot.q = q
                else:
                    now = time.monotonic()
                    if now - last_hold_print[1] > 1.0:
                        print("[IK] R hold", flush=True)
                        last_hold_print[1] = now
                right_controller.T = _swift_se3(frames.right.transform_horizon_to_controller).A
                right_flange.T = _swift_se3(frames.right.transform_horizon_to_flange).A

            if browser_reconnected():
                drain_queues(env)
                _reemit_scene(env, items, is_connected)

            pico_ok = sub.pico_connected(pico_stale_s)
            tracked = sub.tracked
            if not pico_ok:
                status = "PICO: disconnected (waiting...)"
            else:
                lt = "ok" if tracked[0] == 1 else "lost"
                rt = "ok" if tracked[1] == 1 else "lost"
                status = f"PICO: connected | L {lt} | R {rt}"
            if status != last_status:
                status_label.desc = status
                last_status = status

            n += 1
            if n % 50 == 0 and frames.left is not None:
                ctrl = frames.left.transform_horizon_to_controller[:3, 3]
                flange = frames.left.transform_horizon_to_flange[:3, 3]
                print(f"[L] h ctrl={np.round(ctrl, 4)} flange={np.round(flange, 4)}", flush=True)

            env.step(dt)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            env.close()
        except Exception:
            pass
        sub.close()
    return 0
