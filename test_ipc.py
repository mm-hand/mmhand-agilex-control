from __future__ import annotations

import asyncio
import socket
import threading
import time

from ipc import IpcClient, IpcServer, decode, encode, parse_frame


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def test_parse_frame_accepts_valid_csv():
    line = "1.0,1,0.1,0.2,0.3,0,0,0,1,0,0,0,0,0,0,0,1,5,0"
    frame = parse_frame(line)
    assert frame is not None
    assert frame["left"]["valid"] is True
    assert frame["left"]["buttons"] == 5
    assert frame["right"]["buttons"] == 0
    assert frame["left"]["p"] == [0.1, 0.2, 0.3]


def test_parse_frame_rejects_invalid_rows():
    assert parse_frame("1,1,0") is None
    assert parse_frame("1.0,2,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0") is None
    assert parse_frame("nan,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0") is None


def test_ndjson_roundtrip():
    message = {"type": "hello", "protocol": 1, "role": "control"}
    assert decode(encode(message)) == message
    assert decode(b"not json") is None


def test_client_server_roundtrip():
    port = _free_port()
    received = []
    ready = threading.Event()

    async def serve():
        server = IpcServer("127.0.0.1", port, received.append)
        await server.start()
        ready.set()
        await asyncio.sleep(1.0)
        await server.close()

    thread = threading.Thread(target=lambda: asyncio.run(serve()), daemon=True)
    thread.start()
    assert ready.wait(2.0)
    client = IpcClient("127.0.0.1", port, lambda message: None)
    client.start()
    deadline = time.monotonic() + 2.0
    while not client.connected and time.monotonic() < deadline:
        time.sleep(0.01)
    assert client.send({"type": "ping"})
    deadline = time.monotonic() + 2.0
    while not received and time.monotonic() < deadline:
        time.sleep(0.01)
    client.close()
    thread.join(timeout=2.0)
    assert any(message["type"] == "ping" for message in received)
