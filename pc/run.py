#!/usr/bin/env python3
import argparse
import asyncio
import shutil
import subprocess
import sys
import webbrowser
from datetime import datetime
from pathlib import Path

DEVICE_PORT = 29761
PACKAGE = "com.mmhand.pico_head_relative_pose"
ROOT = Path(__file__).resolve().parent.parent
HEADER = "timestamp,left_valid,lx,ly,lz,lqx,lqy,lqz,lqw,right_valid,rx,ry,rz,rqx,rqy,rqz,rqw,left_buttons,right_buttons"


def parse_frame(line):
    fields = line.split(",")
    if len(fields) != 19 or fields[1] not in ("0", "1") or fields[9] not in ("0", "1"):
        return None
    try:
        frame = [float(value) for value in fields[:17]] + [int(value) for value in fields[17:19]]
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
            raise RuntimeError(
                "ROS 2 is not available; load its environment or add --no-ros"
            ) from exc

        rclpy.init(args=[])
        self.rclpy = rclpy
        self.Pose = Pose
        self.PoseArray = PoseArray
        self.UInt8MultiArray = UInt8MultiArray
        self.UInt32MultiArray = UInt32MultiArray
        self.node = rclpy.create_node("read_pico")
        self.poses = self.node.create_publisher(PoseArray, "/read_pico/poses", qos_profile_sensor_data)
        self.buttons = self.node.create_publisher(
            UInt32MultiArray, "/read_pico/buttons", qos_profile_sensor_data
        )
        self.tracked = self.node.create_publisher(
            UInt8MultiArray, "/read_pico/tracked", qos_profile_sensor_data
        )
        print("ROS 2: /read_pico/{poses,buttons,tracked}")

    def publish(self, frame):
        poses = self.PoseArray()
        poses.header.stamp = self.node.get_clock().now().to_msg()
        poses.header.frame_id = "pico_horizon"
        poses.poses = [self._pose(frame, 1), self._pose(frame, 9)]
        self.poses.publish(poses)
        self.buttons.publish(self.UInt32MultiArray(data=[frame[17], frame[18]]))
        self.tracked.publish(self.UInt8MultiArray(data=[int(frame[1]), int(frame[9])]))
        self.rclpy.spin_once(self.node, timeout_sec=0)

    def _pose(self, frame, valid_index):
        pose = self.Pose()
        pose.position.x, pose.position.y, pose.position.z = frame[valid_index + 1 : valid_index + 4]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = frame[
            valid_index + 4 : valid_index + 8
        ]
        return pose

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class State:
    def __init__(self, record, on_frame=None):
        self.line = ""
        self.sequence = self.total = self.clients = 0
        self.rate = 0
        self.file = None
        self.on_frame = on_frame
        if record:
            out = ROOT / "recordings"
            out.mkdir(exist_ok=True)
            self.file = (out / f"poses_{datetime.now():%Y%m%d_%H%M%S}.csv").open(
                "w", encoding="utf-8"
            )
            self.file.write(HEADER + "\n")

    def accept(self, line):
        frame = parse_frame(line)
        if frame is None:
            return
        self.line = line
        self.sequence += 1
        self.total += 1
        if self.file:
            self.file.write(line + "\n")
        if self.on_frame:
            self.on_frame(frame)


def find_adb():
    adb = shutil.which("adb")
    if not adb and sys.platform == "win32":
        bundled = ROOT / "toolchains" / "android-sdk" / "platform-tools" / "adb.exe"
        if bundled.is_file():
            adb = str(bundled)
    if not adb:
        raise RuntimeError("adb was not found in PATH")
    return adb


def adb_setup(vr_port):
    adb = find_adb()
    subprocess.run([adb, "get-state"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run([adb, "reverse", f"tcp:{DEVICE_PORT}", f"tcp:{vr_port}"], check=True)
    subprocess.run(
        [adb, "shell", "monkey", "-p", PACKAGE, "-c", "android.intent.category.LAUNCHER", "1"],
        check=True,
        stdout=subprocess.DEVNULL,
    )
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
    previous = 0
    while True:
        await asyncio.sleep(1)
        state.rate, previous = state.total - previous, state.total
        print(
            f"{state.rate:3d} pose/s | clients={state.clients} | total={state.total}",
            end="\r",
            flush=True,
        )


async def start_web(state, port):
    try:
        from aiohttp import web
    except ImportError as exc:
        raise RuntimeError("visualization requires: python -m pip install aiohttp") from exc

    async def index(_):
        return web.FileResponse(Path(__file__).with_name("view.html"))

    async def socket(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        sent = -1
        try:
            while not ws.closed:
                if state.sequence != sent and state.line:
                    sent = state.sequence
                    try:
                        await ws.send_str(state.line)
                    except ConnectionError:
                        break
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
    url = f"http://127.0.0.1:{port}"
    print(f"Visualizer: {url}")
    webbrowser.open(url)
    return runner


async def main(args):
    ros = None if args.no_ros else RosPublisher()
    state = State(args.record, ros.publish if ros else None)
    runner = monitor = None
    try:
        server = await asyncio.start_server(
            lambda reader, writer: receive(reader, writer, state), "127.0.0.1", args.vr_port
        )
        runner = await start_web(state, args.visualize_port) if args.visualize else None
        adb_setup(args.vr_port)
        print(f"VR data: listening on 127.0.0.1:{args.vr_port}; Ctrl+C to stop")
        monitor = asyncio.create_task(stats(state))
        async with server:
            await server.serve_forever()
    finally:
        if monitor:
            monitor.cancel()
        if runner:
            await runner.cleanup()
        if state.file:
            state.file.close()
        if ros:
            ros.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Receive PICO controller input over USB")
    parser.add_argument(
        "--vr-port", type=int, default=9100, help="PC port receiving PICO data (default: 9100)"
    )
    parser.add_argument("--visualize", action="store_true", help="open browser visualization")
    parser.add_argument(
        "--visualize-port",
        type=int,
        default=8765,
        help="browser HTTP/WebSocket port (default: 8765)",
    )
    parser.add_argument("--no-ros", action="store_true", help="disable ROS 2 publishing")
    parser.add_argument("--record", action="store_true", help="save raw CSV under recordings/")
    args = parser.parse_args(argv)
    for name in ("vr_port", "visualize_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error(f"--{name.replace('_', '-')} must be between 1 and 65535")
    if args.visualize and args.vr_port == args.visualize_port:
        parser.error("--vr-port and --visualize-port must differ when visualization is enabled")
    return args


if __name__ == "__main__":
    try:
        asyncio.run(main(parse_args()))
    except KeyboardInterrupt:
        print("\nstopped")
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        sys.exit(f"error: {exc}")
