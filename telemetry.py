from __future__ import annotations

import logging
import threading
import time
from typing import Callable

import numpy as np

from joint_feedback import FeedbackRateWindow, FeedbackRates


TelemetryValues = tuple[
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray | None,
    np.ndarray | None,
]


def _copy(value: np.ndarray | None) -> np.ndarray | None:
    return None if value is None else np.asarray(value, dtype=float).copy()


def _log_feedback_rates(
    logger: logging.Logger,
    side: str,
    rates: FeedbackRates,
    warn_below_hz: float,
) -> None:
    minimum = min(rates.joint_hz, rates.publish_hz, *rates.motor_hz)
    log = logger.warning if minimum < warn_below_hz else logger.info
    log(
        "feedback rates side=%s get_joint_angles=%.2fHz get_motor_states=%sHz telemetry=%.2fHz",
        side,
        rates.joint_hz,
        np.round(rates.motor_hz, 2).tolist(),
        rates.publish_hz,
    )


class TelemetryPublisher:
    def __init__(
        self,
        pose_input,
        logger: logging.Logger,
        publish_hz: float,
        rate_window_s: float,
        warn_below_hz: float,
        *,
        real_side: str | None = None,
        backend=None,
        current_pose: Callable[[np.ndarray], np.ndarray] | None = None,
    ) -> None:
        if publish_hz <= 0.0:
            raise ValueError("telemetry publish rate must be positive")
        if real_side is not None and (backend is None or current_pose is None):
            raise ValueError("real telemetry requires a backend and current-pose function")
        self.pose_input = pose_input
        self.logger = logger
        self.period_s = 1.0 / publish_hz
        self.poll_period_s = min(0.001, self.period_s) if real_side is not None else self.period_s
        self.warn_below_hz = warn_below_hz
        self.real_side = real_side
        self.backend = backend
        self.current_pose = current_pose
        self.rate_window = FeedbackRateWindow(rate_window_s) if real_side is not None else None
        self.last_published_joint_timestamp = None
        self.values: TelemetryValues = (None,) * 8
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error: Exception | None = None
        self.thread = threading.Thread(target=self._run, name="telemetry-publisher", daemon=True)

    def update(self, *values: np.ndarray | None) -> None:
        if len(values) != 8:
            raise ValueError("telemetry update requires eight values")
        with self.lock:
            self.values = tuple(_copy(value) for value in values)

    def _snapshot(
        self,
        current_q: np.ndarray | None = None,
        current_pose: np.ndarray | None = None,
    ) -> TelemetryValues:
        with self.lock:
            values = list(self.values)
        if self.real_side == "left":
            values[0], values[2] = current_q, current_pose
        elif self.real_side == "right":
            values[4], values[6] = current_q, current_pose
        return tuple(_copy(value) for value in values)

    def _tick(self, now: float) -> None:
        if self.real_side is None:
            self.pose_input.publish_telemetry(*self._snapshot())
            return
        fresh = self.backend.refresh_feedback()
        sample = self.backend.feedback_sample()
        self.rate_window.observe(sample)
        joint_timestamp = sample[1]
        if fresh and joint_timestamp != self.last_published_joint_timestamp:
            current_q = self.backend.current_joints(self.real_side)
            timestamp_arg = {f"{self.real_side}_current_timestamp_s": joint_timestamp}
            self.pose_input.publish_telemetry(
                *self._snapshot(current_q, self.current_pose(current_q)),
                **timestamp_arg,
            )
            self.last_published_joint_timestamp = joint_timestamp
            self.rate_window.mark_published(now)
        rates = self.rate_window.pop(now)
        if rates is not None:
            _log_feedback_rates(
                self.logger, self.real_side, rates, self.warn_below_hz
            )

    def _run(self) -> None:
        deadline = time.monotonic()
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                self._tick(now)
                deadline += self.poll_period_s
                if deadline <= now:
                    deadline = now + self.poll_period_s
                self.stop_event.wait(max(0.0, deadline - time.monotonic()))
        except Exception as exc:
            self.error = exc
            self.logger.exception("telemetry publisher failed")
            self.stop_event.set()

    def start(self) -> None:
        self.thread.start()

    def check(self) -> None:
        if self.error is not None:
            raise RuntimeError("telemetry publisher failed") from self.error

    def close(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
