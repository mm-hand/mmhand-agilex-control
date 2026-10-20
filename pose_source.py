from __future__ import annotations

import math
import threading
import time

from config import IpcConfig, PoseInputConfig
from frames import ControllerPose
from ipc import IpcClient


class IpcPoseSource:
    """Latest PICO frame received from the pico_bridge process."""

    def __init__(self, ipc_config: IpcConfig, input_config: PoseInputConfig) -> None:
        self.input_config = input_config
        self._lock = threading.Lock()
        self._frame: dict | None = None
        self._received_at = 0.0
        self._status: dict = {"adb": "waiting", "rate_hz": 0.0}
        self._count = 0
        self._rate_started = time.monotonic()
        self._rate_hz = 0.0
        self._next_id = 1
        self.client = IpcClient(
            ipc_config.host,
            ipc_config.port,
            self._on_message,
            ipc_config.max_line_bytes,
            ipc_config.reconnect_min_s,
            ipc_config.reconnect_max_s,
        )

    def start(self) -> None:
        self.client.start()

    def _on_message(self, message: dict) -> None:
        kind = message.get("type")
        if kind == "pico_frame":
            now = time.monotonic()
            with self._lock:
                self._frame = message
                self._received_at = now
                self._count += 1
                elapsed = now - self._rate_started
                if elapsed >= 1.0:
                    self._rate_hz = self._count / elapsed
                    self._count = 0
                    self._rate_started = now
        elif kind == "pico_status":
            with self._lock:
                self._status = {
                    "adb": message.get("adb", "unknown"),
                    "rate_hz": float(message.get("rate_hz", 0.0)),
                }

    def _sample(self, side: str) -> ControllerPose | None:
        with self._lock:
            frame = self._frame
            received_at = self._received_at
        if frame is None:
            return None
        now = time.monotonic()
        if now - received_at > self.input_config.max_pose_age_s:
            return None
        sample = frame.get(side)
        if not sample or not sample.get("valid"):
            return None
        return ControllerPose(
            tuple(float(value) for value in sample["p"]),
            tuple(float(value) for value in sample["q"]),
            received_at,
        )

    def get_left(self) -> ControllerPose | None:
        return self._sample("left")

    def get_right(self) -> ControllerPose | None:
        return self._sample("right")

    def buttons(self) -> tuple[int, int]:
        with self._lock:
            frame = self._frame
        if frame is None:
            return (0, 0)
        return (
            int(frame.get("left", {}).get("buttons", 0)),
            int(frame.get("right", {}).get("buttons", 0)),
        )

    def status(self) -> dict:
        with self._lock:
            received_at = self._received_at
            status = dict(self._status)
            rate = self._rate_hz or status.get("rate_hz", 0.0)
        status["connected"] = received_at > 0.0 and time.monotonic() - received_at < self.input_config.disconnect_s
        status["rate_hz"] = round(float(rate), 1)
        return status

    def send_command(self, action: str, **payload) -> bool:
        command = {"type": "command", "id": self._next_id, "action": action}
        command.update(payload)
        self._next_id += 1
        return self.client.send(command)

    def close(self) -> None:
        self.client.close()


class DemoPoseSource:
    """Synthetic controller motion for headless development."""

    def __init__(self, input_config: PoseInputConfig) -> None:
        self.input_config = input_config
        self._started = time.monotonic()

    def start(self) -> None:
        return None

    def _pose(self, side: str) -> ControllerPose:
        t = time.monotonic() - self._started
        sign = -1.0 if side == "left" else 1.0
        xyz = (
            sign * (0.30 + 0.08 * math.sin(t * 0.7)),
            0.10 + 0.06 * math.cos(t * 0.9),
            -0.35 + 0.08 * math.sin(t * 1.1),
        )
        yaw = 0.35 * math.sin(t * 0.5)
        return ControllerPose(xyz, (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)), time.monotonic())

    def get_left(self) -> ControllerPose | None:
        return self._pose("left")

    def get_right(self) -> ControllerPose | None:
        return self._pose("right")

    def buttons(self) -> tuple[int, int]:
        return (0, 0)

    def status(self) -> dict:
        return {"adb": "demo", "connected": True, "rate_hz": 50.0}

    def send_command(self, action: str, **payload) -> bool:
        return True

    def close(self) -> None:
        return None
