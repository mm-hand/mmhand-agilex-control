from types import SimpleNamespace

import numpy as np

from joint_feedback import FeedbackRates
from telemetry import TelemetryPublisher, _log_feedback_rates


class FakePoseInput:
    def __init__(self) -> None:
        self.messages = []

    def publish_telemetry(self, *values, **timestamps) -> None:
        self.messages.append((values, timestamps))


class FakeBackend:
    def __init__(self, side: str = "left") -> None:
        self.side = side
        self.timestamp = 1.0
        self.q = np.arange(7, dtype=float)

    def refresh_feedback(self) -> bool:
        return True

    def feedback_sample(self):
        timestamps = np.full(7, self.timestamp)
        return self.q.copy(), self.timestamp, self.q.copy(), timestamps

    def current_joints(self, side: str):
        assert side == self.side
        return self.q.copy()


def _logger():
    return SimpleNamespace(
        info=lambda *args: None,
        warning=lambda *args: None,
        exception=lambda *args: None,
    )


def test_real_telemetry_publishes_only_new_joint_timestamps():
    pose_input = FakePoseInput()
    backend = FakeBackend()
    publisher = TelemetryPublisher(
        pose_input,
        _logger(),
        200.0,
        5.0,
        180.0,
        real_side="left",
        backend=backend,
        current_pose=lambda q: np.eye(4),
    )
    publisher.update(*([None] * 8))
    publisher._tick(0.0)
    publisher._tick(0.001)
    backend.timestamp += 0.005
    publisher._tick(0.004)
    assert len(pose_input.messages) == 2
    values, timestamps = pose_input.messages[-1]
    assert np.array_equal(values[0], backend.q)
    assert values[4] is None
    assert timestamps == {"left_current_timestamp_s": backend.timestamp}


def test_sim_telemetry_republishes_latest_snapshot():
    pose_input = FakePoseInput()
    publisher = TelemetryPublisher(pose_input, _logger(), 200.0, 5.0, 180.0)
    q = np.arange(7, dtype=float)
    publisher.update(q, q, np.eye(4), np.eye(4), q, q, np.eye(4), np.eye(4))
    publisher._tick(0.0)
    publisher._tick(0.005)
    assert len(pose_input.messages) == 2
    values, timestamps = pose_input.messages[-1]
    assert all(value is not None for value in values)
    assert timestamps == {}


def test_real_telemetry_routes_right_sdk_timestamp():
    pose_input = FakePoseInput()
    backend = FakeBackend("right")
    publisher = TelemetryPublisher(
        pose_input,
        _logger(),
        200.0,
        5.0,
        180.0,
        real_side="right",
        backend=backend,
        current_pose=lambda q: np.eye(4),
    )
    publisher._tick(0.0)
    values, timestamps = pose_input.messages[0]
    assert values[0] is None
    assert np.array_equal(values[4], backend.q)
    assert timestamps == {"right_current_timestamp_s": backend.timestamp}


def test_feedback_rate_log_warns_below_threshold_and_otherwise_informs():
    calls = []
    logger = SimpleNamespace(
        info=lambda *args: calls.append(("info", args)),
        warning=lambda *args: calls.append(("warning", args)),
    )
    healthy = FeedbackRates(200.0, (200.0,) * 7, 199.0)
    slow = FeedbackRates(200.0, (170.0,) + (200.0,) * 6, 199.0)
    _log_feedback_rates(logger, "left", healthy, warn_below_hz=180.0)
    _log_feedback_rates(logger, "left", slow, warn_below_hz=180.0)
    assert [level for level, _ in calls] == ["info", "warning"]
