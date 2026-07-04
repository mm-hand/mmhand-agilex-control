# analytic_control

Pose-input teleoperation pipeline for Nero dual-arm IK, joint-space motion planning, and Swift visualization.

## Install

ROS 2 Jazzy must already be installed. The pose input publisher must publish:

```text
/read_pose/poses      geometry_msgs/PoseArray
/read_pose/tracked    std_msgs/UInt8MultiArray
```

Install Python dependencies:

```bash
cd /home/starcycle/nero_control/analytic_control
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m pip install --user --break-system-packages -r requirements.txt
```

## Build Cython IK

The analytic IK solver lives in `ik_solver.pyx`. Build it before running:

```bash
cd /home/starcycle/nero_control/analytic_control
/usr/bin/python3 setup.py build_ext --inplace
```

This creates generated C and `.so` files that are ignored by git. Do not edit generated files; edit `ik_solver.pyx` and rebuild.

## Configure

All runtime parameters are centralized in `config.py`:

- `RuntimeConfig`: mode, currently `dual_sim` by default; hardware modes are reserved.
- `PoseInputConfig`: ROS node name, `/read_pose/*` topics, freshness timeouts.
- `FrameTransformConfig`: shoulder/base/tool calibration and human-to-robot scale.
- `IkConfig`: analytic IK branch scoring and arm-angle search settings.
- `FilterConfig`: One Euro filtering for input pose only.
- `MotionConfig`: fixed command frequency plus joint velocity/acceleration limits for `jtraj`.
- `LogConfig`: terminal logging is always enabled; file logging can be disabled here.

## Run

Start the pose input publisher first, then run:

```bash
cd /home/starcycle/nero_control/analytic_control
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 run.py
```

Open the Swift URL printed by the process, typically `http://localhost:52000`.

Logs are printed to the terminal. If `LogConfig.enable_file_log` is true, logs are also written under `logs/` inside this repository.

## Motion Policy

The IK solver outputs target joint angles. `motion.py` is the only layer that sends joint commands to simulation or future hardware:

1. It runs at `MotionConfig.command_hz`.
2. It plans joint-space trajectories with `roboticstoolbox.jtraj`.
3. It chooses trajectory duration from configured joint velocity and acceleration limits.
4. It inherits current joint velocity when replanning, avoiding repeated stop-start motion.
5. It replans only when the target moves beyond `replan_trigger_rad` and `min_replan_interval_s` has elapsed.
6. If IK fails, it holds the current command and clears active trajectory.

## Test

```bash
cd /home/starcycle/nero_control/analytic_control
/usr/bin/python3 setup.py build_ext --inplace
/usr/bin/python3 test_ik.py
/usr/bin/python3 test_motion.py
```

## Files

- `run.py`: thin entry point
- `config.py`: all runtime configuration
- `control_loop.py`: pose input -> frame transform -> IK -> motion -> backend loop
- `pose_io.py`: ROS subscribers for pose input
- `frames.py`: pose-input-to-arm frame transforms
- `ik.py`: Nero IK wrapper and URDF robot loader
- `ik_solver.pyx`: Cython analytic arm-angle IK solver
- `motion.py`: fixed-rate joint-space trajectory manager
- `filters.py`: One Euro pose filter
- `sim.py`: Swift visualization backend
- `backends.py`: backend protocol and reserved hardware backend
- `logging_utils.py`: terminal and optional file logging
- `test_ik.py`, `test_motion.py`: verification tests
- `nero_urdf/`: local URDF and meshes
