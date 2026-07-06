# analytic_control

Pose-input teleoperation pipeline for Nero analytic IK, online joint-space motion limiting, Swift simulation, and single-arm real control.

## Install

Use a conda/RoboStack ROS environment. Do not source `/opt/ros/jazzy/setup.bash` when this environment is active.

```bash
conda create -y -n mmhand-ros --override-channels   -c conda-forge -c robostack-jazzy   python=3.12   ros-jazzy-ros-base   ros-jazzy-geometry-msgs   ros-jazzy-std-msgs   ros-jazzy-rmw-cyclonedds-cpp
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
python -m pip install -r requirements.txt
```

The pose input publisher must publish:

```text
/read_pose/poses      geometry_msgs/PoseArray
/read_pose/tracked    std_msgs/UInt8MultiArray
```

## Build IK

```bash
python setup.py build_ext --inplace
```

This builds `ik_solver.pyx`. Generated C and shared-object files are ignored by git.

## Configure

Runtime parameters live in `config.py`:

- `RuntimeConfig`: `dual_sim`, `left_real`, `right_real`.
- `PoseInputConfig`: ROS node name, pose/tracked topics, freshness timeouts.
- `FrameTransformConfig`: shoulder, base, controller, and flange calibration.
- `IkConfig`: arm-angle search and continuous-branch scoring.
- `InputLimiterConfig`: controller pose speed and per-tick step limits.
- `FilterConfig`: OneEuro filters for pose input and IK-output joint targets.
- `StartupConfig`: left-arm startup pose for simulation, with real-arm move_j disabled by default.
- `MotionConfig`: online joint velocity, acceleration, step, and limit margins.
- `SimConfig`: Swift visualization offset and console joint-angle log period.
- `RealArmConfig`: `pyAgxArm` path, SocketCAN channel, speed, and timeouts.

## Run

```bash
python run.py
```

`RuntimeConfig.mode = "dual_sim"` opens Swift, typically at `http://localhost:52000`. For real-arm modes, place the vendor SDK at `third_party/pyAgxArm`, set SocketCAN up, then set `RuntimeConfig.mode` to `left_real` or `right_real`:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Logs are printed to stdout.

## Runtime Policy

The loop is intentionally small: optional left-arm startup pose -> ROS pose input -> OneEuro pose filter -> input limiter -> frame calibration -> analytic IK -> OneEuro joint-target filter -> online joint motion limiter -> Swift or real-arm backend. Missing pose/IK targets reset the IK-output filter, hold the internal motion planner, and send no new real-arm command. Real hardware is controlled with `move_js` after live joint feedback is available.

## Files

- `run.py`: entry point and stdout logger
- `config.py`: runtime configuration
- `control_loop.py`: teleoperation loop
- `pose_io.py`: ROS pose subscribers
- `frames.py`: controller-to-arm frame transforms
- `ik.py`: Nero IK wrapper and URDF loader
- `ik_solver.pyx`: Cython analytic IK solver
- `motion.py`: online joint-space limiter
- `filters.py`: OneEuro pose/joint filters and input pose limiter
- `sim.py`: Swift backend
- `hardware.py`: `pyAgxArm` real-arm backend
- `nero_urdf/`: URDF and meshes
