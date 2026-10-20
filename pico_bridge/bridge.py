from __future__ import annotations

import asyncio
import logging
import os
import shutil
import subprocess
import time
from pathlib import Path

from config import PicoConfig
from ipc import IpcServer, parse_frame
from pico_bridge.ros_output import PicoRosOutput


def find_adb(config: PicoConfig) -> str:
    candidates: list[str | Path | None] = [config.adb_path or None]
    local_app_data = os.environ.get("LOCALAPPDATA")
    if local_app_data:
        candidates.append(Path(local_app_data) / "Android" / "Sdk" / "platform-tools" / "adb.exe")
    candidates.extend(
        [
            shutil.which("adb"),
            Path.home() / "Android" / "Sdk" / "platform-tools" / "adb",
            Path.home() / "Android" / "Sdk" / "platform-tools" / "adb.exe",
        ]
    )
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(candidate)
    raise RuntimeError("adb not found; set PicoConfig.adb_path or add adb to PATH")


def adb_setup(config: PicoConfig) -> None:
    adb = find_adb(config)
    subprocess.run([adb, "get-state"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0)
    subprocess.run(
        [adb, "reverse", f"tcp:{config.device_port}", f"tcp:{config.vr_port}"],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=5.0,
    )
    subprocess.run(
        [adb, "shell", "monkey", "-p", config.package, "-c", "android.intent.category.LAUNCHER", "1"],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=5.0,
    )


def adb_remove_reverse(config: PicoConfig) -> None:
    try:
        adb = find_adb(config)
        subprocess.run(
            [adb, "reverse", "--remove", f"tcp:{config.device_port}"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
        )
    except (OSError, RuntimeError, subprocess.SubprocessError):
        pass


class PicoBridge:
    def __init__(self, config: PicoConfig, logger: logging.Logger, no_adb: bool = False) -> None:
        self.config = config
        self.logger = logger
        self.no_adb = no_adb
        self.ros = PicoRosOutput(logger)
        self._stop = asyncio.Event()
        self._adb_ready = False
        self._adb_error: str | None = None
        self._frames = 0
        self._rate_hz = 0.0
        self._rate_started = time.monotonic()
        self._tasks: list[asyncio.Task] = []
        self._ipc: IpcServer | None = None
        self._server: asyncio.AbstractServer | None = None

    async def run(self) -> None:
        self._ipc = IpcServer(self.config.ipc_host, self.config.ipc_port, self._on_ipc_message)
        await self._ipc.start()
        self._server = await asyncio.start_server(self._handle_ingest, self.config.ingest_host, self.config.vr_port)
        self._tasks = [
            asyncio.create_task(self._adb_loop(), name="adb-loop"),
            asyncio.create_task(self._stats_loop(), name="stats-loop"),
        ]
        self.logger.info("pico bridge ready: vr=%s ipc=%s", self.config.vr_port, self.config.ipc_port)
        await self._stop.wait()

    async def _handle_ingest(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while not reader.at_eof():
                line = await reader.readline()
                if not line:
                    break
                frame = parse_frame(line.decode("ascii", "ignore"))
                if frame is None:
                    continue
                self._frames += 1
                if self._ipc is not None:
                    await self._ipc.send(frame, latest=True)
                self.ros.publish(frame)
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass

    def _on_ipc_message(self, message: dict) -> None:
        if message.get("type") == "command":
            asyncio.create_task(self._handle_command(message))

    async def _handle_command(self, message: dict) -> None:
        action = message.get("action")
        command_id = message.get("id", 0)
        ok, error = True, None
        if action == "set_ros_output":
            ok = self.ros.set_enabled(bool(message.get("enabled")))
            error = None if ok else self.ros.reason
        elif action == "reconnect_adb":
            self._adb_ready = False
        elif action == "shutdown":
            self._stop.set()
        else:
            ok, error = False, f"unknown action: {action}"
        if self._ipc is not None:
            await self._ipc.send({"type": "command_ack", "id": command_id, "ok": ok, "error": error})

    async def _adb_loop(self) -> None:
        if self.no_adb:
            self._adb_error = "adb disabled"
            return
        while not self._stop.is_set():
            if not self._adb_ready:
                try:
                    await asyncio.to_thread(adb_setup, self.config)
                    self._adb_ready = True
                    self._adb_error = None
                    self.logger.info("ADB reverse ready: device %s -> PC %s", self.config.device_port, self.config.vr_port)
                except Exception as exc:
                    message = str(exc)
                    if message != self._adb_error:
                        self.logger.warning("ADB setup failed: %s", message)
                    self._adb_error = message
            await asyncio.sleep(self.config.adb_retry_s)

    async def _stats_loop(self) -> None:
        period = 1.0 / max(0.5, self.config.status_hz)
        while not self._stop.is_set():
            await asyncio.sleep(period)
            now = time.monotonic()
            elapsed = max(1e-6, now - self._rate_started)
            self._rate_hz = self._frames / elapsed
            self._frames = 0
            self._rate_started = now
            if self._ipc is not None:
                await self._ipc.send(self._status())

    def _status(self) -> dict:
        if self.no_adb:
            adb = "disabled"
        elif self._adb_ready:
            adb = "connected"
        elif self._adb_error:
            adb = "error"
        else:
            adb = "waiting"
        return {"type": "pico_status", "adb": adb, "rate_hz": round(self._rate_hz, 1)}

    def stop(self) -> None:
        self._stop.set()

    async def close(self) -> None:
        self._stop.set()
        for task in self._tasks:
            task.cancel()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        if self._ipc is not None:
            await self._ipc.close()
            self._ipc = None
        self.ros.close()
        if not self.no_adb:
            await asyncio.to_thread(adb_remove_reverse, self.config)
