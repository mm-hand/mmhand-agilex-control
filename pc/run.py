#!/usr/bin/env python3
import argparse, asyncio, errno, os, shutil, signal, subprocess, sys
from pathlib import Path

DEVICE_PORT = 29761
PACKAGE = "com.mmhand.pico_head_relative_pose"


def parse_frame(line):
    f = line.split(",")
    if len(f) != 19 or f[1] not in ("0", "1") or f[9] not in ("0", "1"):
        return None
    try:
        frame = [float(v) for v in f[:17]] + [int(v) for v in f[17:19]]
    except ValueError:
        return None
    return frame if frame[17] >= 0 and frame[18] >= 0 else None


class RosPublisher:
    def __init__(self):
        try:
            import rclpy
            from geometry_msgs.msg import Pose, PoseArray
            from rclpy.qos import qos_profile_sensor_data
            from std_msgs.msg import UInt8MultiArray, UInt32MultiArray
        except ImportError as exc:
            raise RuntimeError("ROS 2 not available") from exc
        rclpy.init(args=[])
        self.rclpy = rclpy
        self.Pose, self.PoseArray = Pose, PoseArray
        self.U8, self.U32 = UInt8MultiArray, UInt32MultiArray
        self.node = rclpy.create_node("read_pico")
        self.poses = self.node.create_publisher(PoseArray, "/read_pico/poses", qos_profile_sensor_data)
        self.buttons = self.node.create_publisher(UInt32MultiArray, "/read_pico/buttons", qos_profile_sensor_data)
        self.tracked = self.node.create_publisher(UInt8MultiArray, "/read_pico/tracked", qos_profile_sensor_data)
        print("ROS 2: /read_pico/{poses,buttons,tracked}")

    def publish(self, frame):
        poses = self.PoseArray()
        poses.header.stamp = self.node.get_clock().now().to_msg()
        poses.header.frame_id = "pico_horizon"
        for i, vi in enumerate((1, 9)):
            p = self.Pose()
            p.position.x, p.position.y, p.position.z = frame[vi + 1:vi + 4]
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = frame[vi + 4:vi + 8]
            poses.poses.append(p)
        if not self.rclpy.ok():
            return
        try:
            self.poses.publish(poses)
            self.buttons.publish(self.U32(data=[frame[17], frame[18]]))
            self.tracked.publish(self.U8(data=[int(frame[1]), int(frame[9])]))
            self.rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            if not self.rclpy.ok():
                return
            raise

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok(): self.rclpy.shutdown()


class State:
    def __init__(self, on_frame=None):
        self.line = ""
        self.sequence = self.total = self.clients = 0
        self.on_frame = on_frame

    def accept(self, line):
        frame = parse_frame(line)
        if frame is None: return
        self.line = line
        self.sequence += 1
        self.total += 1
        if self.on_frame: self.on_frame(frame)


def find_adb():
    for adb in (Path.home() / "Android" / "Sdk" / "platform-tools" / "adb", shutil.which("adb")):
        if adb and Path(adb).is_file(): return str(adb)
    raise RuntimeError("adb not found")


def adb_setup(vr_port):
    adb = find_adb()
    subprocess.run([adb, "get-state"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run([adb, "reverse", f"tcp:{DEVICE_PORT}", f"tcp:{vr_port}"], check=True)
    subprocess.run([adb, "shell", "monkey", "-p", PACKAGE, "-c", "android.intent.category.LAUNCHER", "1"], check=True, stdout=subprocess.DEVNULL)
    print(f"ADB reverse: PICO tcp:{DEVICE_PORT} -> PC tcp:{vr_port}")


async def receive(reader, writer, state):
    state.clients += 1
    print(f"PICO connected: {writer.get_extra_info('peername')}")
    try:
        while data := await reader.readline():
            state.accept(data.decode("ascii", "ignore").strip())
    finally:
        state.clients -= 1
        writer.close()
        await writer.wait_closed()
        print("PICO disconnected; waiting for reconnection")


async def stats(state):
    prev = 0
    while True:
        await asyncio.sleep(1)
        rate, prev = state.total - prev, state.total
        print(f"{rate:3d} pose/s | clients={state.clients} | total={state.total}", end="\r", flush=True)


async def start_web(state, port):
    from aiohttp import web
    async def index(_): return web.FileResponse(Path(__file__).with_name("view.html"))
    async def socket(request):
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        sent = -1
        try:
            while not ws.closed:
                if state.sequence != sent and state.line:
                    sent = state.sequence
                    try: await ws.send_str(state.line)
                    except ConnectionError: break
                await asyncio.sleep(0.01)
        finally:
            await ws.close()
        return ws
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", socket)
    runner = web.AppRunner(app, access_log=None)
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", port).start()
    print(f"Visualizer: http://127.0.0.1:{port}")
    return runner


def free_port(port):
    import re
    pids = set()
    try:
        out = subprocess.run(["ss", "-ltnpH"], capture_output=True, text=True, check=False)
        for m in re.finditer(r"pid=(\d+)", out.stdout):
            if f":{port} " in out.stdout:
                pids.add(m.group(1))
    except FileNotFoundError:
        pass
    try:
        out = subprocess.run(["lsof", "-ti", f":{port}"], capture_output=True, text=True, check=False)
        pids.update(p for p in out.stdout.split() if p)
    except FileNotFoundError:
        pass
    for pid in pids:
        if pid != str(os.getpid()):
            try:
                os.kill(int(pid), signal.SIGTERM)
                print(f"killed pid {pid} occupying port {port}")
            except (ProcessLookupError, ValueError, PermissionError):
                pass
    return bool(pids)


async def main(args):
    ros = RosPublisher() if os.environ.get("READ_PICO_ROS") == "1" else None
    state = State(ros.publish if ros else None)
    runner = monitor = None
    try:
        try:
            server = await asyncio.start_server(lambda r, w: receive(r, w, state), "127.0.0.1", args.vr_port)
        except OSError as exc:
            if exc.errno == errno.EADDRINUSE and free_port(args.vr_port):
                await asyncio.sleep(0.3)
                server = await asyncio.start_server(lambda r, w: receive(r, w, state), "127.0.0.1", args.vr_port)
            else:
                raise
        runner = None if ros else await start_web(state, args.visualize_port)
        adb_setup(args.vr_port)
        print(f"VR data: listening on 127.0.0.1:{args.vr_port}; Ctrl+C to stop")
        monitor = asyncio.create_task(stats(state))
        async with server: await server.serve_forever()
    finally:
        if monitor: monitor.cancel()
        if runner: await runner.cleanup()
        if ros:
            state.on_frame = None
            ros.close()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Receive PICO controller input over USB")
    p.add_argument("--vr-port", type=int, default=9100)
    p.add_argument("--visualize-port", type=int, default=8765)
    return p.parse_args(argv)


if __name__ == "__main__":
    try:
        asyncio.run(main(parse_args()))
    except KeyboardInterrupt:
        print("\nstopped")
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        sys.exit(f"error: {exc}")
