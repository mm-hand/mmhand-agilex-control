from __future__ import annotations

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
