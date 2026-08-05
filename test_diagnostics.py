from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np

from config import ControlConfig
from diagnostics import RunDiagnostics


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _diagnostics(tmp_path):
    config = ControlConfig(log=replace(ControlConfig().log, directory=str(tmp_path)))
    clock = Clock()
    diagnostics = RunDiagnostics(config, clock)
    diagnostics.set_fk(lambda q: np.array([
        [1.0, 0.0, 0.0, q[0]], [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0],
    ]))
    return diagnostics, clock


def _feedback(q, fresh=True):
    return {
        "q": np.asarray(q, dtype=float), "velocity": np.zeros(7),
        "current": np.ones(7), "torque": np.zeros(7),
        "timestamps": np.ones(7), "source": "motor_states", "fresh": fresh,
    }


def test_smooth_command_is_accepted(tmp_path):
    diagnostics, clock = _diagnostics(tmp_path)
    q = np.zeros(7)
    diagnostics.record("startup", command_mode="move_j", command_q=q)
    clock.advance(0.02)
    diagnostics.mark_enter()
    clock.advance(0.02)
    diagnostics.record("handoff", command_mode="move_js", command_q=q)
    for value in (0.0004, 0.0012):
        clock.advance(0.02)
        q = np.full(7, value)
        diagnostics.record("teleop", command_mode="move_js", command_q=q)
    assert diagnostics.summary()["command_continuous"] is True
    diagnostics.close(SimpleNamespace(info=lambda *args: None, warning=lambda *args: None))


def test_command_jump_is_detected(tmp_path):
    diagnostics, clock = _diagnostics(tmp_path)
    diagnostics.record("startup", command_mode="move_j", command_q=np.zeros(7))
    clock.advance(0.02)
    diagnostics.mark_enter()
    clock.advance(0.02)
    diagnostics.record("teleop", ik_q=np.ones(7), command_mode="move_js", command_q=np.full(7, 0.1))
    assert diagnostics.summary()["command_continuous"] is False


def test_ik_jump_does_not_fail_a_smooth_command(tmp_path):
    diagnostics, clock = _diagnostics(tmp_path)
    q = np.zeros(7)
    diagnostics.record("startup", command_mode="move_j", command_q=q)
    clock.advance(0.02)
    diagnostics.mark_enter()
    clock.advance(0.02)
    diagnostics.record("handoff", command_mode="move_js", command_q=q)
    clock.advance(0.02)
    diagnostics.record("teleop", ik_q=np.full(7, 0.5), command_mode="move_js", command_q=np.full(7, 0.0004))
    clock.advance(0.02)
    diagnostics.record("teleop", ik_q=np.full(7, -0.5), command_mode="move_js", command_q=np.full(7, 0.0012))
    summary = diagnostics.summary()
    assert summary["ik_continuous"] is False
    assert summary["command_continuous"] is True


def test_actual_jump_and_persistent_tracking_error_are_detected(tmp_path):
    diagnostics, clock = _diagnostics(tmp_path)
    diagnostics.record("teleop", command_q=np.zeros(7), feedback=_feedback(np.zeros(7)))
    clock.advance(0.21)
    diagnostics.record("teleop", command_q=np.zeros(7), feedback=_feedback(np.full(7, 0.2)))
    clock.advance(0.21)
    diagnostics.record("teleop", command_q=np.zeros(7), feedback=_feedback(np.full(7, 0.2)))
    summary = diagnostics.summary()
    assert summary["actual_continuous"] is False
    assert summary["tracking_safe"] is False
    assert summary["feedback_valid"] is True


def test_stale_feedback_is_not_treated_as_real_motion(tmp_path):
    diagnostics, clock = _diagnostics(tmp_path)
    diagnostics.record("teleop", feedback=_feedback(np.zeros(7), fresh=False))
    clock.advance(0.02)
    diagnostics.record("teleop", feedback=_feedback(np.zeros(7), fresh=False))
    summary = diagnostics.summary()
    assert summary["feedback_valid"] is False
    assert summary["actual_continuous"] is None
