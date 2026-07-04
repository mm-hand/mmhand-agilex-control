from __future__ import annotations

from typing import Protocol

import numpy as np

from frames import CoordinateFrames


class Backend(Protocol):
    def update(
        self,
        left_q: np.ndarray | None,
        right_q: np.ndarray | None,
        frames: CoordinateFrames,
        status: str,
        dt: float,
    ) -> None: ...

    def close(self) -> None: ...


class HardwareBackend:
    def __init__(self, mode: str) -> None:
        self.mode = mode
        raise NotImplementedError(f"hardware backend is reserved but not implemented: {mode}")
