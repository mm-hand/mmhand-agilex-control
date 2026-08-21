from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np


JointFeedbackSample = tuple[np.ndarray, float, np.ndarray, np.ndarray]


def read_feedback_sample(robot) -> JointFeedbackSample | None:
    try:
        joint_packet = robot.get_joint_angles()
        motor_packets = [robot.get_motor_states(joint) for joint in range(1, 8)]
    except Exception:
        return None
    if (
        joint_packet is None
        or joint_packet.msg is None
        or any(packet is None or packet.msg is None for packet in motor_packets)
    ):
        return None
    joint_q = np.asarray(joint_packet.msg, dtype=float)
    motor_q = np.asarray([packet.msg.position for packet in motor_packets], dtype=float)
    motor_timestamps = np.asarray([packet.timestamp for packet in motor_packets], dtype=float)
    joint_timestamp = float(joint_packet.timestamp)
    if joint_q.shape != (7,) or not all(
        np.all(np.isfinite(values))
        for values in (joint_q, motor_q, motor_timestamps, [joint_timestamp])
    ):
        return None
    return joint_q, joint_timestamp, motor_q, motor_timestamps


def timestamp_hz(timestamps) -> float:
    unique = np.unique(np.asarray(timestamps, dtype=float))
    unique = unique[np.isfinite(unique)]
    if unique.size < 2 or unique[-1] <= unique[0]:
        return 0.0
    return float((unique.size - 1) / (unique[-1] - unique[0]))


@dataclass(frozen=True)
class FeedbackRates:
    joint_hz: float
    motor_hz: tuple[float, ...]
    publish_hz: float


class FeedbackRateWindow:
    def __init__(self, window_s: float) -> None:
        if window_s <= 0.0:
            raise ValueError("feedback rate window must be positive")
        self.window_s = float(window_s)
        self.reset()

    def reset(self, now: float | None = None) -> None:
        self.started_at = time.monotonic() if now is None else float(now)
        self.joint_timestamps: list[float] = []
        self.motor_timestamps: list[list[float]] = [[] for _ in range(7)]
        self.publish_times: list[float] = []

    @staticmethod
    def _append_changed(values: list[float], value: float) -> None:
        if not values or value != values[-1]:
            values.append(value)

    def observe(self, sample: JointFeedbackSample) -> None:
        _, joint_timestamp, _, motor_timestamps = sample
        self._append_changed(self.joint_timestamps, joint_timestamp)
        for values, timestamp in zip(self.motor_timestamps, motor_timestamps):
            self._append_changed(values, float(timestamp))

    def mark_published(self, now: float | None = None) -> None:
        self.publish_times.append(time.monotonic() if now is None else float(now))

    def pop(self, now: float | None = None) -> FeedbackRates | None:
        current = time.monotonic() if now is None else float(now)
        if current - self.started_at < self.window_s:
            return None
        rates = FeedbackRates(
            joint_hz=timestamp_hz(self.joint_timestamps),
            motor_hz=tuple(timestamp_hz(values) for values in self.motor_timestamps),
            publish_hz=timestamp_hz(self.publish_times),
        )
        self.reset(current)
        return rates
