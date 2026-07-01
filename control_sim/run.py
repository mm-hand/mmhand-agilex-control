from __future__ import annotations

import argparse

from .config import FrameTransformConfig, IkConfig, RobotControlConfig, RosConfig, SimConfig


def main() -> int:
    argparse.ArgumentParser(description="PICO -> nero dual-arm IK + Swift sim").parse_args()
    from .sim import run_sim
    return run_sim(RosConfig(), FrameTransformConfig(), IkConfig(), SimConfig(), RobotControlConfig())


if __name__ == "__main__":
    raise SystemExit(main())
