from __future__ import annotations

import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from config import ControlConfig
from joint_feedback import read_feedback_sample as _snapshot
from joint_feedback import timestamp_hz as _timestamp_hz


def _connect_passive(config):
    sdk_path = Path(config.sdk_path)
    if not sdk_path.is_absolute():
        sdk_path = Path(__file__).resolve().parent / sdk_path
    if not sdk_path.exists():
        raise RuntimeError(f"pyAgxArm SDK not found: {sdk_path}")
    if str(sdk_path) not in sys.path:
        sys.path.insert(0, str(sdk_path))
    from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

    sdk_config = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=NeroFW.DEFAULT,
        interface=config.interface,
        channel=config.channel,
        bitrate=config.bitrate,
    )
    robot = AgxArmFactory.create_arm(sdk_config)
    try:
        robot.connect()
    except Exception:
        robot.disconnect()
        raise
    return robot


def _ensure_can_ready(config) -> None:
    if config.interface != "socketcan":
        return
    net_dir = Path("/sys/class/net") / config.channel
    if not net_dir.exists():
        raise RuntimeError(f"SocketCAN interface {config.channel!r} does not exist")
    flags = int((net_dir / "flags").read_text(encoding="utf-8").strip(), 16)
    if not flags & 0x1:
        raise RuntimeError(f"SocketCAN interface {config.channel!r} is DOWN")


def _wait_for_snapshot(robot, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = _snapshot(robot)
        if snapshot is not None:
            return snapshot
        time.sleep(0.01)
    raise RuntimeError("timed out waiting for joint-angle and motor-state feedback")


def _capture(robot, path: Path, first_snapshot) -> None:
    fields = [
        "elapsed_s",
        "joint_timestamp",
        *(f"joint_q{joint}" for joint in range(1, 8)),
        *(f"motor_timestamp_q{joint}" for joint in range(1, 8)),
        *(f"motor_q{joint}" for joint in range(1, 8)),
    ]
    started = time.monotonic()
    previous_joint_timestamp = None
    previous_motor_timestamps = None
    previous_joint_q = None
    previous_motor_q = None
    snapshot = first_snapshot
    with path.open("w", newline="", encoding="utf-8", buffering=1) as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        while True:
            if snapshot is not None:
                joint_q, joint_timestamp, motor_q, motor_timestamps = snapshot
                changed = (
                    joint_timestamp != previous_joint_timestamp
                    or previous_motor_timestamps is None
                    or np.any(motor_timestamps != previous_motor_timestamps)
                    or previous_joint_q is None
                    or np.any(joint_q != previous_joint_q)
                    or np.any(motor_q != previous_motor_q)
                )
                if changed:
                    writer.writerow([
                        time.monotonic() - started,
                        joint_timestamp,
                        *joint_q,
                        *motor_timestamps,
                        *motor_q,
                    ])
                    previous_joint_timestamp = joint_timestamp
                    previous_motor_timestamps = motor_timestamps.copy()
                    previous_joint_q = joint_q.copy()
                    previous_motor_q = motor_q.copy()
            time.sleep(0.001)
            snapshot = _snapshot(robot)


def _plot(data: np.ndarray, path: Path) -> None:
    figure, axes = plt.subplots(7, 1, sharex=True, figsize=(10, 14))
    for index, axis in enumerate(axes, start=1):
        axis.plot(data["elapsed_s"], data[f"joint_q{index}"], label="get_joint_angles", linewidth=1.0)
        axis.plot(data["elapsed_s"], data[f"motor_q{index}"], label="get_motor_states", linewidth=1.0)
        axis.set_ylabel(f"J{index}\nrad")
        axis.grid(alpha=0.3)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("Elapsed time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _report(data: np.ndarray) -> None:
    joint_hz = _timestamp_hz(data["joint_timestamp"])
    motor_hz = np.asarray([
        _timestamp_hz(data[f"motor_timestamp_q{joint}"]) for joint in range(1, 8)
    ])
    print(f"get_joint_angles timestamp rate: {joint_hz:.2f} Hz")
    print(f"get_motor_states timestamp rates: {np.round(motor_hz, 2).tolist()} Hz")
    print(f"get_motor_states conservative rate: {np.min(motor_hz):.2f} Hz")


def main() -> int:
    config = ControlConfig()
    _ensure_can_ready(config.real)
    robot = _connect_passive(config.real)
    try:
        first_snapshot = _wait_for_snapshot(robot, config.real.startup_timeout_s)
        output_dir = Path(config.log.directory) / "joint_feedback"
        output_dir.mkdir(parents=True, exist_ok=True)
        stem = output_dir / f"{datetime.now():%Y%m%d_%H%M%S}_joint_feedback"
        csv_path = stem.with_suffix(".csv")
        print(f"passive feedback capture started: {csv_path}")
        print("Press Ctrl+C to stop and generate the plot.")
        try:
            _capture(robot, csv_path, first_snapshot)
        except KeyboardInterrupt:
            print("\ncapture stopped")
    finally:
        robot.disconnect()

    data = np.atleast_1d(np.genfromtxt(csv_path, delimiter=",", names=True))
    if data.size == 0:
        raise RuntimeError("no feedback samples were recorded")
    plot_path = stem.with_suffix(".png")
    _plot(data, plot_path)
    _report(data)
    print(f"CSV: {csv_path}")
    print(f"plot: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
