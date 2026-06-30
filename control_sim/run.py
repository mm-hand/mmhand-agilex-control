from __future__ import annotations

import argparse

from .config import (
    FrameTransformConfig,
    IkConfig,
    RobotControlConfig,
    RosConfig,
    SimConfig,
    VisualizationConfig,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PICO -> nero dual-arm mapping, IK, and visualization.")
    parser.add_argument("--mode", choices=("frames", "sim"), default="frames",
                        help="frames = matplotlib coordinate frames; sim = swift browser with IK-driven nero arms")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ros_config = RosConfig()
    frame_config = FrameTransformConfig()

    if args.mode == "frames":
        from .viz_frames import run_frames
        return run_frames(ros_config, frame_config, VisualizationConfig())

    from .sim import run_sim
    return run_sim(
        ros_config,
        frame_config,
        IkConfig(),
        SimConfig(),
        RobotControlConfig(),
    )


if __name__ == "__main__":
    raise SystemExit(main())
