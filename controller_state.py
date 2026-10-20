from __future__ import annotations

import threading
from dataclasses import dataclass, field


SIDES = ("left", "right")


@dataclass(frozen=True)
class ControllerSnapshot:
    valid: bool
    xyz: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    buttons: int = 0


@dataclass(frozen=True)
class ArmStatus:
    state: str = "SIM"
    channel: str = ""
    error: str | None = None


@dataclass(frozen=True)
class ControlSnapshot:
    phase: str = "STARTING"
    pico: dict = field(default_factory=dict)
    controllers: dict[str, ControllerSnapshot | None] = field(
        default_factory=lambda: {side: None for side in SIDES}
    )
    sim_q: dict[str, tuple[float, ...]] = field(
        default_factory=lambda: {side: (0.0,) * 7 for side in SIDES}
    )
    real_q: dict[str, tuple[float, ...] | None] = field(
        default_factory=lambda: {side: None for side in SIDES}
    )
    arm_status: dict[str, ArmStatus] = field(
        default_factory=lambda: {side: ArmStatus() for side in SIDES}
    )
    ros: dict = field(default_factory=dict)
    handoff_id: int = 0
    handoff_sides: tuple[str, ...] = ()
    error: str | None = None


@dataclass(frozen=True)
class ControlCommand:
    kind: str
    side: str = ""
    channel: str = ""
    enabled: bool = False
    request_id: int = 0


class SnapshotStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot = ControlSnapshot()

    def publish(self, snapshot: ControlSnapshot) -> None:
        with self._lock:
            self._snapshot = snapshot

    def read(self) -> ControlSnapshot:
        with self._lock:
            return self._snapshot
