# analytic_control

Pose-input teleoperation pipeline for Nero analytic IK, online joint-space safety motion, Swift visualization, and single-arm real control.

## Install

Use a conda/RoboStack ROS environment. Do not source `/opt/ros/jazzy/setup.bash`
when this environment is active.

```bash
conda create -y -n mmhand-ros --override-channels \
  -c conda-forge -c robostack-jazzy \
  python=3.12 \
  ros-jazzy-ros-base \
  ros-jazzy-geometry-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-rmw-cyclonedds-cpp

conda activate mmhand-ros
export PYTHONNOUSERSITE=1
```

The pose input publisher must publish:

```text
/read_pose/poses      geometry_msgs/PoseArray
/read_pose/tracked    std_msgs/UInt8MultiArray
```

Install Python dependencies:

```bash
cd /home/starcycle/nero_control/analytic_control
python -m pip install -r requirements.txt

# RoboStack currently provides NumPy 2.x. Rebuild Swift Cython extensions
# inside this environment so they match the local NumPy ABI.
python -m pip install --force-reinstall --no-cache-dir \
  --no-binary=spatialgeometry --no-build-isolation --no-deps \
  spatialgeometry==1.1.0
python -m pip install --force-reinstall --no-cache-dir \
  --no-binary=swift-sim --no-build-isolation --no-deps \
  swift-sim==1.1.0
```

## Build Cython IK

The analytic IK solver lives in `ik_solver.pyx`. Build it before running:

```bash
cd /home/starcycle/nero_control/analytic_control
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
python setup.py build_ext --inplace
```

This creates generated C and `.so` files that are ignored by git. Do not edit generated files; edit `ik_solver.pyx` and rebuild.

## Configure

All runtime parameters are centralized in `config.py`:

- `RuntimeConfig`: `dual_sim`, `left_real`, `right_real`; `dual_real` is reserved.
- `PoseInputConfig`: ROS node name, `/read_pose/*` topics, freshness timeouts.
- `FrameTransformConfig`: shoulder/base/tool calibration and human-to-robot scale.
- `IkConfig`: analytic IK scoring, branch switching gate, and arm-angle search.
- `FilterConfig`: One Euro pose filter plus VR input speed/step limiter.
- `MotionConfig`: online joint servo limits plus startup `jtraj` duration limits.
- `RealArmConfig`: `pyAgxArm` SDK path, SocketCAN, startup pose, SDK speed, and send timing.
- `LogConfig`: terminal logging is always enabled; file logging can be disabled here.

## Run

Start the pose input publisher first, then run:

```bash
cd /home/starcycle/nero_control/analytic_control
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
python run.py
```

`RuntimeConfig.mode = "dual_sim"` opens Swift, typically at `http://localhost:52000`.
For real-arm modes, place the vendor SDK at `third_party/pyAgxArm`, set SocketCAN up first, then set `RuntimeConfig.mode` to `left_real` or `right_real`:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Logs are printed to the terminal. If `LogConfig.enable_file_log` is true, logs are also written under `logs/` inside this repository.


## IK Policy

The continuous IK score uses joint velocity, joint acceleration, pose error, arm-angle continuity, and joint-limit soft cost. Reach clamp is a feature: it saturates unreachable wrist targets before IK.

The first solve may use a global arm-angle search to acquire a branch. Later solves score all local candidates, but a branch change is accepted only when the maximum single-joint jump is below `branch_switch_max_joint_delta_rad` and the score improvement exceeds `branch_switch_min_improvement`. If IK fails, no new real-arm command is sent.

## Motion Policy

`motion.py` is the final software safety layer between IK and sim/real output:

1. Every control tick follows the newest IK target, not an old cached goal.
2. Joint velocity, joint acceleration, per-tick joint step, and inner joint limits are enforced.
3. Missing IK target zeros commanded velocity and holds the current internal command.
4. Real hardware receives no new command when the target is missing.
5. `roboticstoolbox.jtraj` is kept only for startup or deliberate large recovery moves, not continuous teleop.

## Test

```bash
cd /home/starcycle/nero_control/analytic_control
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
python setup.py build_ext --inplace
python test_ik.py
python test_motion.py
python test_hardware.py
```

## Files

- `run.py`: thin entry point
- `config.py`: all runtime configuration
- `control_loop.py`: pose input -> frame transform -> IK -> motion -> backend loop
- `pose_io.py`: ROS subscribers for pose input
- `frames.py`: pose-input-to-arm frame transforms
- `ik.py`: Nero IK wrapper and URDF robot loader
- `ik_solver.pyx`: Cython analytic arm-angle IK solver
- `motion.py`: online joint-space servo and startup trajectory helper
- `filters.py`: One Euro pose filter and input limiter
- `sim.py`: Swift visualization backend
- `hardware.py`: `pyAgxArm` real-arm backend
- `logging_utils.py`: terminal and optional file logging
- `test_ik.py`, `test_motion.py`, `test_hardware.py`: verification tests
- `nero_urdf/`: local URDF and meshes
