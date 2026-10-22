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


def adb_reverse_active(config: PicoConfig) -> bool:
    """True when the headset ingest tunnel is currently registered."""
    adb = find_adb(config)
    result = subprocess.run(
        [adb, "reverse", "--list"],
        check=True,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    return f"tcp:{config.device_port}" in (result.stdout or "")


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


# PICO system overlays that grab input focus while the VR app keeps running.
# `HandDialogActivity` opens on its own virtual display and `PicoXR_CustomDialog`
# is the window that actually covers the VR view; both come from the system ext
# package and neither answers BACK.
SYSTEM_OVERLAY_TOKENS = ("HandDialogActivity", "PicoXR_CustomDialog")

def current_focus_windows(text: str) -> list[str]:
    """Windows actually holding input focus, per display, skipping idle ones."""
    windows = []
    for line in text.splitlines():
        if "mCurrentFocus=" not in line:
            continue
        window = line.split("mCurrentFocus=", 1)[1].split("mFocusedApp=")[0].strip()
        if window.startswith("null"):
            continue
        windows.append(window)
    return windows

def focus_stealer(text: str) -> str | None:
    """Name of the PICO system overlay holding window focus, if any.

    Only ``mCurrentFocus`` counts: ``mFocusedApp`` keeps a stale ActivityRecord
    for the dialog after its window is gone, and acting on that would force-stop
    the overlay package over and over.
    """
    for window in current_focus_windows(text):
        for token in SYSTEM_OVERLAY_TOKENS:
            if token in window:
                return token
    return None

def read_focus(config: PicoConfig) -> str:
    adb = find_adb(config)
    result = subprocess.run(
        [adb, "shell", "dumpsys window | grep -E 'mCurrentFocus|mFocusedApp'"],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    return result.stdout or ""


def restore_focus(config: PicoConfig) -> None:
    """Dismiss the system overlay that took focus away from the VR app.

    Measured on a PICO with the dialog showing: `am start --activity-single-top`
    reports "intent has been delivered to currently running top-most instance"
    and changes nothing, and injected BACK/ESCAPE keys land on display 0 while
    the dialog owns a virtual display of its own. Force-stopping the host process
    is the only action that actually returns focus, and it does so immediately.
    """
    adb = find_adb(config)
    subprocess.run(
        [adb, "shell", "am", "force-stop", config.overlay_package],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=5.0,
    )


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
            asyncio.create_task(self._focus_loop(), name="focus-loop"),
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
            try:
                if self._adb_ready and await asyncio.to_thread(adb_reverse_active, self.config):
                    pass
                else:
                    if self._adb_ready:
                        self.logger.warning("ADB reverse lost; reconnecting")
                        self._adb_ready = False
                    await asyncio.to_thread(adb_setup, self.config)
                    self._adb_ready = True
                    self._adb_error = None
                    self.logger.info("ADB reverse ready: device %s -> PC %s", self.config.device_port, self.config.vr_port)
            except Exception as exc:
                message = str(exc)
                if message != self._adb_error:
                    self.logger.warning("ADB setup failed: %s", message)
                self._adb_error = message
                self._adb_ready = False
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

    async def _focus_loop(self) -> None:
        if not self.config.focus_guard:
            return
        last_error: str | None = None
        last_restore = 0.0
        while not self._stop.is_set():
            await asyncio.sleep(self.config.focus_guard_interval_s)
            try:
                text = await asyncio.to_thread(read_focus, self.config)
                stealer = focus_stealer(text)
                if stealer is None:
                    last_error = None
                    continue
                now = time.monotonic()
                if now - last_restore < self.config.focus_guard_min_interval_s:
                    continue
                last_restore = now
                self.logger.warning("PICO overlay %s stole focus; dismissing", stealer)
                await asyncio.to_thread(restore_focus, self.config)
                if focus_stealer(await asyncio.to_thread(read_focus, self.config)) is None:
                    self.logger.info("PICO focus restored after dismissing %s", stealer)
                else:
                    self.logger.warning("PICO overlay %s still holds focus", stealer)
                last_error = None
            except Exception as exc:
                message = str(exc)
                if message != last_error:
                    self.logger.warning("focus guard failed: %s", message)
                    last_error = message

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
