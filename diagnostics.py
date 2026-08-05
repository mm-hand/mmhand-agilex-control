from __future__ import annotations

import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Callable

import numpy as np

from config import ControlConfig
from frames import ControllerPose
from motion import angle_diff


FIELDS = (
    "wall_time", "elapsed_s", "since_enter_s", "phase", "event", "command_mode",
    "pico_raw", "pico_filtered", "ik_q", "command_q", "actual_q",
    "motor_velocity", "motor_current", "motor_torque", "motor_timestamps",
    "feedback_source", "feedback_fresh", "target_flange", "ik_flange",
    "command_flange", "actual_flange", "command_step_max_rad",
    "command_velocity_max_rad_s", "command_acceleration_max_rad_s2",
    "actual_step_max_rad", "actual_velocity_max_rad_s",
    "actual_acceleration_max_rad_s2", "tracking_error_max_rad",
    "actual_position_step_m", "actual_rotation_step_deg", "note",
)


def _array(value) -> np.ndarray | None:
    if value is None:
        return None
    result = np.asarray(value, dtype=float)
    return result if np.all(np.isfinite(result)) else None


def _pose(value: ControllerPose | None) -> np.ndarray | None:
    return None if value is None else np.asarray((*value.xyz_m, *value.quat_xyzw), dtype=float)


def _compact(value) -> str:
    value = _array(value)
    return "" if value is None else json.dumps(value.tolist(), separators=(",", ":"))


def _rotation_step(a: np.ndarray, b: np.ndarray) -> float:
    cosine = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


class RunDiagnostics:
    def __init__(self, config: ControlConfig, clock: Callable[[], float] = time.monotonic) -> None:
        self.config, self.clock = config, clock
        self.started, self.entered_at = clock(), None
        self.fk: Callable[[np.ndarray], np.ndarray] | None = None
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        directory = Path(config.log.directory) / f"{stamp}_{config.runtime.mode}"
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / "telemetry.csv"
        self.output = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.output, fieldnames=FIELDS)
        self.writer.writeheader()
        self.last_flush = self.started
        self.write_error = None
        self.closed = False

        self.previous_command = self.previous_command_velocity = None
        self.previous_command_at = None
        self.previous_actual = self.previous_actual_velocity = self.previous_actual_flange = None
        self.previous_actual_at = None
        self.previous_ik = None
        self.command_ok = self.ik_ok = self.actual_ok = self.tracking_ok = True
        self.command_count = self.ik_count = self.actual_count = 0
        self.command_comparison_count = 0
        self.teleop_feedback_count = self.teleop_fresh_count = 0
        self.tracking_bad_since = None
        self.first_ik_gap = self.max_tracking_error = 0.0
        self.windows: dict[str, dict[str, float]] = {}
        self.record("startup", event="run_start")

    def set_fk(self, function: Callable[[np.ndarray], np.ndarray]) -> None:
        self.fk = function

    def mark_enter(self) -> None:
        self.entered_at = self.clock()
        self.previous_command_at = self.entered_at
        self.previous_command_velocity = np.zeros(7)
        self.previous_actual_at = self.entered_at
        self.previous_actual_velocity = np.zeros(7)
        self.command_ok = self.actual_ok = self.tracking_ok = True
        self.command_count = self.actual_count = 0
        self.command_comparison_count = 0
        self.teleop_feedback_count = self.teleop_fresh_count = 0
        self.tracking_bad_since = None
        self.record("handoff", event="enter_pressed")

    def _window(self, now: float) -> str:
        if self.entered_at is None:
            return "pre-enter"
        elapsed = now - self.entered_at
        for upper, name in ((0.25, "0-0.25s"), (0.5, "0.25-0.5s"), (1.0, "0.5-1s"), (2.0, "1-2s")):
            if elapsed < upper:
                return name
        return "2s+"

    def _motion_metrics(self, now: float, q: np.ndarray, actual: bool):
        previous = self.previous_actual if actual else self.previous_command
        previous_v = self.previous_actual_velocity if actual else self.previous_command_velocity
        previous_at = self.previous_actual_at if actual else self.previous_command_at
        step = velocity = acceleration = None
        if previous is not None and previous_at is not None and now > previous_at:
            dt = now - previous_at
            step = angle_diff(q, previous)
            velocity = step / dt
            if previous_v is not None:
                acceleration = (velocity - previous_v) / dt
        if actual:
            self.previous_actual, self.previous_actual_velocity, self.previous_actual_at = q.copy(), velocity, now
        else:
            self.previous_command, self.previous_command_velocity, self.previous_command_at = q.copy(), velocity, now
        maximum = lambda value: None if value is None else float(np.max(np.abs(value)))
        return step, velocity, acceleration, maximum(step), maximum(velocity), maximum(acceleration)

    def record(
        self, phase: str, *, event: str = "sample", command_mode: str = "",
        pico_raw=None, pico_filtered=None, ik_q=None, command_q=None,
        feedback: dict | None = None, target_flange=None, note: str = "",
    ) -> None:
        now = self.clock()
        command, ik = _array(command_q), _array(ik_q)
        feedback = feedback or {}
        actual = _array(feedback.get("q"))
        fresh = bool(feedback.get("fresh", False))
        command_step = command_velocity = command_acceleration = None
        actual_step = actual_velocity = actual_acceleration = None
        command_step_max = command_velocity_max = command_acceleration_max = None
        actual_step_max = actual_velocity_max = actual_acceleration_max = None
        actual_position_step = actual_rotation_step = tracking_error = None

        if command_mode and command is not None:
            command_step, command_velocity, command_acceleration, command_step_max, command_velocity_max, command_acceleration_max = self._motion_metrics(now, command, False)
            self.command_count += 1
            if command_step is not None:
                self.command_comparison_count += 1
                self.command_ok &= bool(np.all(np.abs(command_step) <= self.config.motion.max_joint_step_rad + 1e-6))
            if command_velocity is not None:
                self.command_ok &= bool(np.all(np.abs(command_velocity) <= np.asarray(self.config.motion.max_velocity_rad_s) + 1e-3))
            if command_acceleration is not None:
                self.command_ok &= bool(np.all(np.abs(command_acceleration) <= np.asarray(self.config.motion.max_acceleration_rad_s2) + 1e-2))

        if ik is not None:
            self.ik_count += 1
            if self.previous_ik is None and self.previous_command is not None:
                self.first_ik_gap = float(np.max(np.abs(angle_diff(ik, self.previous_command))))
            elif self.previous_ik is not None:
                jump = float(np.max(np.abs(angle_diff(ik, self.previous_ik))))
                self.ik_ok &= jump <= self.config.ik.branch_switch_max_joint_delta_rad + 1e-6
            self.previous_ik = ik.copy()

        if phase in {"handoff", "teleop"} and actual is not None:
            self.teleop_feedback_count += 1
            self.teleop_fresh_count += int(fresh)
        actual_flange = self.fk(actual) if self.fk and actual is not None else None
        if actual is not None and fresh:
            actual_step, actual_velocity, actual_acceleration, actual_step_max, actual_velocity_max, actual_acceleration_max = self._motion_metrics(now, actual, True)
            self.actual_count += 1
            if actual_flange is not None and self.previous_actual_flange is not None:
                actual_position_step = float(np.linalg.norm(actual_flange[:3, 3] - self.previous_actual_flange[:3, 3]))
                actual_rotation_step = _rotation_step(self.previous_actual_flange, actual_flange)
            self.previous_actual_flange = actual_flange
            if actual_velocity is not None:
                limits = 1.2 * np.asarray(self.config.motion.max_velocity_rad_s)
                self.actual_ok &= bool(np.all(np.abs(actual_velocity) <= limits))
            if actual_acceleration is not None:
                limits = 1.2 * np.asarray(self.config.motion.max_acceleration_rad_s2)
                self.actual_ok &= bool(np.all(np.abs(actual_acceleration) <= limits))
            self.actual_ok &= actual_position_step is None or actual_position_step <= 0.025
            self.actual_ok &= actual_rotation_step is None or actual_rotation_step <= 5.0

        if command is not None and actual is not None and fresh:
            tracking_error = float(np.max(np.abs(angle_diff(command, actual))))
            self.max_tracking_error = max(self.max_tracking_error, tracking_error)
            if tracking_error > 0.15:
                if self.tracking_bad_since is None:
                    self.tracking_bad_since = now
                elif now - self.tracking_bad_since >= 0.2:
                    self.tracking_ok = False
            else:
                self.tracking_bad_since = None

        window = self.windows.setdefault(self._window(now), {})
        for name, value in (
            ("command_step_rad", command_step_max), ("command_velocity_rad_s", command_velocity_max),
            ("command_acceleration_rad_s2", command_acceleration_max),
            ("actual_step_rad", actual_step_max), ("actual_velocity_rad_s", actual_velocity_max),
            ("actual_acceleration_rad_s2", actual_acceleration_max),
            ("tracking_error_rad", tracking_error), ("position_step_m", actual_position_step),
            ("rotation_step_deg", actual_rotation_step),
        ):
            if value is not None:
                window[name] = max(window.get(name, 0.0), value)

        command_flange = None
        if self.fk and command is not None and not (phase == "teleop" and event == "command_sent"):
            command_flange = self.fk(command)
        row = {
            "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "elapsed_s": now - self.started,
            "since_enter_s": "" if self.entered_at is None else now - self.entered_at,
            "phase": phase, "event": event, "command_mode": command_mode,
            "pico_raw": _compact(_pose(pico_raw)), "pico_filtered": _compact(_pose(pico_filtered)),
            "ik_q": _compact(ik), "command_q": _compact(command), "actual_q": _compact(actual),
            "motor_velocity": _compact(feedback.get("velocity")), "motor_current": _compact(feedback.get("current")),
            "motor_torque": _compact(feedback.get("torque")), "motor_timestamps": _compact(feedback.get("timestamps")),
            "feedback_source": feedback.get("source", ""), "feedback_fresh": int(fresh) if actual is not None else "",
            "target_flange": _compact(target_flange), "ik_flange": _compact(self.fk(ik) if self.fk and ik is not None else None),
            "command_flange": _compact(command_flange),
            "actual_flange": _compact(actual_flange), "command_step_max_rad": command_step_max,
            "command_velocity_max_rad_s": command_velocity_max, "command_acceleration_max_rad_s2": command_acceleration_max,
            "actual_step_max_rad": actual_step_max, "actual_velocity_max_rad_s": actual_velocity_max,
            "actual_acceleration_max_rad_s2": actual_acceleration_max, "tracking_error_max_rad": tracking_error,
            "actual_position_step_m": actual_position_step, "actual_rotation_step_deg": actual_rotation_step, "note": note,
        }
        if self.writer is not None:
            try:
                self.writer.writerow(row)
                if now - self.last_flush >= 1.0:
                    self.output.flush()
                    self.last_flush = now
            except Exception as exc:
                self.write_error = exc
                self.writer = None
                try:
                    self.output.close()
                except Exception:
                    pass

    def summary(self) -> dict[str, bool | float | str | None]:
        feedback_valid = self.teleop_feedback_count >= 2 and self.teleop_fresh_count / self.teleop_feedback_count >= 0.95
        return {
            "ik_continuous": self.ik_ok if self.ik_count >= 2 else None,
            "command_continuous": self.command_ok if self.command_comparison_count else None,
            "actual_continuous": self.actual_ok if feedback_valid and self.actual_count >= 2 else None,
            "tracking_safe": self.tracking_ok if feedback_valid and self.actual_count >= 2 else None,
            "feedback_valid": feedback_valid,
            "first_ik_gap_rad": self.first_ik_gap,
            "max_tracking_error_rad": self.max_tracking_error,
            "csv": str(self.path),
        }

    def close(self, logger) -> None:
        if self.closed:
            return
        self.closed = True
        if self.writer is not None:
            self.record("shutdown", event="run_end")
            self.output.close()
        if self.write_error is not None:
            logger.warning("telemetry logging stopped after error: %s", self.write_error)
        summary = self.summary()
        for key in ("ik_continuous", "command_continuous", "actual_continuous", "tracking_safe", "feedback_valid"):
            value = summary[key]
            label = "UNKNOWN" if value is None else "YES" if value else "NO"
            (logger.info if value else logger.warning)("%s: %s", key.replace("_", " "), label)
        logger.info(
            "diagnostics first_ik_gap=%.4frad max_tracking_error=%.4frad csv=%s",
            self.first_ik_gap, self.max_tracking_error, self.path,
        )
        if not summary["feedback_valid"]:
            cause = "feedback invalid; physical jump cannot be determined"
        elif summary["command_continuous"] is False:
            cause = "sent command discontinuity; inspect IK and planner output"
        elif summary["actual_continuous"] is False or summary["tracking_safe"] is False:
            cause = "sent commands are smooth but the arm lags or jumps; inspect move_js/controller tracking"
        elif summary["ik_continuous"] is False:
            cause = "IK jumps were present but the final sent commands remained smooth"
        else:
            cause = "no Enter-transition discontinuity detected"
        (logger.info if cause.startswith("no ") else logger.warning)("diagnosis: %s", cause)
        for name, values in self.windows.items():
            logger.info("diagnostics window=%s metrics=%s", name, json.dumps(values, sort_keys=True))
