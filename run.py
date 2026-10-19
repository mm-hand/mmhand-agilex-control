from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

try:
    import ik_solver  # noqa: F401
except ImportError as exc:
    raise SystemExit("ik_solver is not built; run scripts/build_ik.ps1 or scripts/build_ik.sh") from exc

from config import ControlConfig
from control_loop import ArmController
from controller_state import SnapshotStore
from logging_utils import LogBuffer, configure_logging
from pose_source import DemoPoseSource, IpcPoseSource
from run_log import SessionLogger

ROOT = Path(__file__).resolve().parent


def find_free_port(host: str, preferred: int, span: int = 50) -> int:
    for port in range(preferred, preferred + span):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            if os.name == "posix":
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind((host, port))
            except OSError:
                continue
            return port
    raise RuntimeError(f"no free port on {host} in {preferred}-{preferred + span - 1}")


class PicoSupervisor:
    def __init__(self, config, vr_port: int, ipc_port: int, buffer: LogBuffer, logger, disabled: bool) -> None:
        self.config = config
        self.vr_port = vr_port
        self.ipc_port = ipc_port
        self.buffer = buffer
        self.logger = logger
        self.disabled = disabled
        self._stop = threading.Event()
        self._process: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self.disabled:
            self.buffer.append("pico", "disabled")
            return
        self._thread = threading.Thread(target=self._run, name="pico-supervisor", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        attempts = 0
        while not self._stop.is_set() and attempts < 3:
            command = [
                sys.executable,
                "-m",
                "pico_bridge",
                "--vr-port",
                str(self.vr_port),
                "--ipc-port",
                str(self.ipc_port),
                "--managed",
            ]
            env = dict(os.environ, PYTHONUNBUFFERED="1")
            try:
                self._process = subprocess.Popen(
                    command,
                    cwd=str(ROOT),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    env=env,
                )
            except OSError as exc:
                self.logger.warning("failed to start pico bridge: %s", exc)
                return
            assert self._process.stdout is not None
            for line in self._process.stdout:
                self.buffer.append("pico", line)
            self._process.wait()
            if self._stop.is_set():
                return
            attempts += 1
            self.logger.warning("pico bridge exited (code %s); restart %s/3", self._process.returncode, attempts)
            time.sleep(1.0)

    def stop(self) -> None:
        self._stop.set()
        process = self._process
        if process is not None and process.poll() is None:
            try:
                if process.stdin is not None:
                    process.stdin.close()
                process.wait(timeout=1.5)
            except (OSError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="AgileX Nero Teleop")
    parser.add_argument("--no-pico", action="store_true", help="do not start the pico bridge")
    parser.add_argument("--demo", action="store_true", help="use synthetic controller motion")
    parser.add_argument("--headless", action="store_true", help="run without NiceGUI")
    parser.add_argument("--duration", type=float, default=0.0, help="headless run time in seconds")
    parser.add_argument("--port", type=int, default=0, help="NiceGUI port")
    parser.add_argument("--vr-port", type=int, default=0, help="PICO ingest port")
    parser.add_argument("--ipc-port", type=int, default=0, help="IPC port")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
    parser.add_argument("--ros-output", action="store_true", help="enable ROS 2 output at startup")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    os.chdir(ROOT)
    config = ControlConfig()
    no_pico = args.no_pico or args.demo
    vr_port = find_free_port(config.pico.ingest_host, args.vr_port or config.pico.vr_port)
    ipc_port = find_free_port(config.ipc.host, args.ipc_port or config.ipc.port)
    ui_port = find_free_port(config.gui.host, args.port or config.gui.port, config.gui.port_span)
    if args.ros_output:
        config = replace(config, ros_output=replace(config.ros_output, enabled_default=True))
    buffer = LogBuffer()
    session = SessionLogger(config.log, "teleop")
    logger = configure_logging(config.log, session.text_log_path, buffer)
    supervisor = PicoSupervisor(config, vr_port, ipc_port, buffer, logger, no_pico)
    supervisor.start()
    if args.demo:
        pose_source = DemoPoseSource(config.pose_input)
    else:
        pose_source = IpcPoseSource(replace(config.ipc, port=ipc_port), config.pose_input)
    pose_source.start()
    store = SnapshotStore()
    controller = None
    try:
        controller = ArmController(config, logger, session, pose_source, store)
        controller.start()
        if args.headless:
            logger.info("headless control started; Ctrl+C to stop")
            if args.duration > 0:
                time.sleep(args.duration)
            else:
                while True:
                    time.sleep(1.0)
        else:
            from ui.app import run_ui

            run_ui(controller, store, config, buffer, ui_port, show=not args.no_browser)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if controller is not None:
            controller.stop()
            controller.wait(timeout=5.0)
        pose_source.send_command("shutdown")
        pose_source.close()
        supervisor.stop()
        session.close(logger)


if __name__ == "__main__":
    raise SystemExit(main())
