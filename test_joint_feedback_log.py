from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from joint_feedback import FeedbackRateWindow
from log_joint_feedback import _snapshot, _timestamp_hz


class FakeRobot:
    def __init__(self, missing_motor: bool = False) -> None:
        self.missing_motor = missing_motor

    def get_joint_angles(self):
        return SimpleNamespace(msg=np.arange(7, dtype=float), timestamp=10.0)

    def get_motor_states(self, joint: int):
        if self.missing_motor and joint == 7:
            return None
        return SimpleNamespace(
            msg=SimpleNamespace(position=0.1 * joint),
            timestamp=20.0 + joint,
        )


def test_timestamp_rate_uses_distinct_sdk_timestamps():
    assert np.isclose(_timestamp_hz([1.0, 1.0, 1.01, 1.02]), 100.0)
    assert _timestamp_hz([1.0, 1.0]) == 0.0


def test_snapshot_maps_both_feedback_sources():
    joint_q, joint_timestamp, motor_q, motor_timestamps = _snapshot(FakeRobot())
    assert np.array_equal(joint_q, np.arange(7, dtype=float))
    assert joint_timestamp == 10.0
    assert np.allclose(motor_q, np.arange(1, 8) * 0.1)
    assert np.array_equal(motor_timestamps, np.arange(21.0, 28.0))


def test_snapshot_rejects_incomplete_motor_feedback():
    assert _snapshot(FakeRobot(missing_motor=True)) is None


def test_rate_window_counts_unique_sdk_timestamps():
    window = FeedbackRateWindow(window_s=1.0)
    window.reset(now=0.0)
    sample = _snapshot(FakeRobot())
    window.observe(sample)
    window.observe(sample)
    window.mark_published(now=0.0)
    window.mark_published(now=0.005)
    rates = window.pop(now=1.0)
    assert rates.joint_hz == 0.0
    assert rates.motor_hz == (0.0,) * 7
    assert np.isclose(rates.publish_hz, 200.0)


def test_snapshot_treats_sdk_read_errors_as_missing_feedback():
    robot = FakeRobot()
    robot.get_joint_angles = lambda: (_ for _ in ()).throw(RuntimeError("read failed"))
    assert _snapshot(robot) is None


if __name__ == "__main__":
    test_timestamp_rate_uses_distinct_sdk_timestamps()
    test_snapshot_maps_both_feedback_sources()
    test_snapshot_rejects_incomplete_motor_feedback()
    print("All joint feedback log tests passed.")
