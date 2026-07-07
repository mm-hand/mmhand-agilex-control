from __future__ import annotations
import numpy as np
from config import ControlConfig
from hardware import create_robot, enable_normal_push, ensure_socketcan_ready, read_current_joint_angles


def main() -> int:
    config = ControlConfig().real
    ensure_socketcan_ready(config)
    robot = create_robot(config)
    try:
        if not enable_normal_push(robot, config.enable_timeout_s):
            raise RuntimeError("timed out while enabling/opening CAN feedback push")
        q = read_current_joint_angles(robot, config.startup_timeout_s)
        print(np.array2string(q, precision=6, suppress_small=True))
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
