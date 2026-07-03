from __future__ import annotations

import argparse
import os
from pathlib import Path

from config import FrameTransformConfig, IkConfig, RosConfig, SimConfig


def main() -> int:
    argparse.ArgumentParser(description="PICO -> nero dual-arm IK + Swift sim").parse_args()
    os.chdir(Path(__file__).resolve().parent)

    from sim import run_sim

    return run_sim(RosConfig(), FrameTransformConfig(), IkConfig(), SimConfig())


if __name__ == "__main__":
    raise SystemExit(main())
