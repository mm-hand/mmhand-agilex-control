from __future__ import annotations
import logging
import numpy as np
from config import ControlConfig
from hardware import connect_robot, read_current_joint_angles


def main() -> int:
    config = ControlConfig().real
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    robot, _ = connect_robot(config, logging.getLogger("read_joints"))
    try:
        q = read_current_joint_angles(robot, config.startup_timeout_s)
        print(np.array2string(q, precision=6, suppress_small=True))
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
