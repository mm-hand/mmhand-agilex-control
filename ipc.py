from __future__ import annotations

import asyncio
import json
import math
import socket
import threading
from typing import Callable

PROTOCOL_VERSION = 1
DEFAULT_MAX_LINE = 65536


def parse_frame(line: str) -> dict | None:
    fields = line.strip().split(",")
    if len(fields) != 19 or fields[1] not in ("0", "1") or fields[9] not in ("0", "1"):
        return None
    try:
        values = [float(value) for value in fields[:17]]
        buttons = [int(value) for value in fields[17:19]]
    except ValueError:
        return None
    if any(not math.isfinite(value) for value in values):
        return None
    if any(value < 0 or value > 0xFFFFFFFF for value in buttons):
        return None
    result = {"type": "pico_frame", "timestamp": values[0]}
    for index, side in ((1, "left"), (9, "right")):
        result[side] = {
            "valid": bool(int(fields[index])),
            "p": values[index + 1 : index + 4],
            "q": values[index + 4 : index + 8],
            "buttons": buttons[0 if side == "left" else 1],
        }
    return result


def encode(message: dict) -> bytes:
    return (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")


def decode(line: bytes) -> dict | None:
    try:
        message = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    return message if isinstance(message, dict) and isinstance(message.get("type"), str) else None


class _Client:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.reader = reader
        self.writer = writer
        self.queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=32)
        self.task: asyncio.Task | None = None

    async def write_loop(self) -> None:
        try:
            while True:
                message = await self.queue.get()
                self.writer.write(encode(message))
                await self.writer.drain()
        except (asyncio.CancelledError, ConnectionError, OSError):
            return


class IpcServer:
    """Async duplex NDJSON server used by the PICO bridge."""

    def __init__(
        self,
        host: str,
        port: int,
        on_message: Callable[[dict], None],
        max_line_bytes: int = DEFAULT_MAX_LINE,
    ) -> None:
        self.host = host
        self.port = port
        self.on_message = on_message
        self.max_line_bytes = max_line_bytes
        self._server: asyncio.AbstractServer | None = None
        self._client: _Client | None = None
        self._lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        return self._client is not None

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle, self.host, self.port)

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        client = _Client(reader, writer)
        async with self._lock:
            old = self._client
            self._client = client
        if old is not None:
            await self._drop(old)
        client.task = asyncio.create_task(client.write_loop())
        try:
            while not reader.at_eof():
                line = await reader.readline()
                if not line:
                    break
                if len(line) > self.max_line_bytes:
                    continue
                message = decode(line)
                if message is not None:
                    self.on_message(message)
        finally:
            async with self._lock:
                if self._client is client:
                    self._client = None
            await self._drop(client)

    async def _drop(self, client: _Client) -> None:
        if client.task is not None:
            client.task.cancel()
        client.writer.close()
        try:
            await client.writer.wait_closed()
        except Exception:
            pass

    async def send(self, message: dict, latest: bool = False) -> None:
        client = self._client
        if client is None:
            return
        if latest and client.queue.qsize() >= 2:
            return
        try:
            client.queue.put_nowait(message)
        except asyncio.QueueFull:
            pass

    async def close(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        async with self._lock:
            client, self._client = self._client, None
        if client is not None:
            await self._drop(client)


class IpcClient:
    """Reconnecting thread client used by the control process."""

    def __init__(
        self,
        host: str,
        port: int,
        on_message: Callable[[dict], None],
        max_line_bytes: int = DEFAULT_MAX_LINE,
        reconnect_min_s: float = 0.5,
        reconnect_max_s: float = 2.0,
    ) -> None:
        self.host = host
        self.port = port
        self.on_message = on_message
        self.max_line_bytes = max_line_bytes
        self.reconnect_min_s = reconnect_min_s
        self.reconnect_max_s = reconnect_max_s
        self._socket: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="ipc-client", daemon=True)
        self._connected = threading.Event()

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def start(self) -> None:
        self._thread.start()

    def send(self, message: dict) -> bool:
        sock = self._socket
        if sock is None:
            return False
        try:
            with self._send_lock:
                sock.sendall(encode(message))
            return True
        except OSError:
            return False

    def _run(self) -> None:
        delay = self.reconnect_min_s
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=1.0)
                sock.settimeout(0.5)
                self._socket = sock
                self._connected.set()
                self.send({"type": "hello", "protocol": PROTOCOL_VERSION, "role": "control"})
                delay = self.reconnect_min_s
                buffer = b""
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    buffer += chunk
                    if len(buffer) > self.max_line_bytes and b"\n" not in buffer:
                        buffer = b""
                        continue
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        if len(line) > self.max_line_bytes:
                            buffer = b""
                            break
                        message = decode(line)
                        if message is not None:
                            self.on_message(message)
            except OSError:
                pass
            finally:
                self._connected.clear()
                sock = self._socket
                self._socket = None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            self._stop.wait(delay)
            delay = min(self.reconnect_max_s, delay * 2)

    def close(self) -> None:
        self._stop.set()
        sock = self._socket
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
