# analytic_control

Nero mechanical-arm analytic IK control for PICO pose teleoperation. It can
run either dual-arm Swift simulation or teleoperate one real arm.

## Install ROS environment

Install ADB for the companion `read_pico` PICO receiver:

```bash
sudo apt install -y adb
```

Create the shared RoboStack Jazzy environment. `analytic_control` and
`read_pico` use this same environment:

```bash
conda create -y -n mmhand-ros --override-channels \
  -c conda-forge -c robostack-jazzy \
  python=3.12 \
  ros-jazzy-ros-base \
  ros-jazzy-geometry-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-rmw-cyclonedds-cpp \
  numpy=1.26 \
  scipy=1.13 \
  cython \
  setuptools \
  wheel \
  websockets=10.4 \
  python-can \
  pyusb \
  roboticstoolbox-python=1.3.0 \
  spatialmath-python=1.1.16 \
  spatialgeometry=1.1.0 \
  swift-sim=1.1.0

conda activate mmhand-ros
export PYTHONNOUSERSITE=1
```

Do not source `/opt/ros/jazzy/setup.bash` while this Conda environment is
active. `PYTHONNOUSERSITE=1` keeps both projects on the same Conda and ROS
packages.

## Prepare source

Initialize the real-arm SDK after cloning:

```bash
cd analytic_control
git submodule update --init --recursive
```

## Configure mode

Edit `RuntimeConfig.mode` in `config.py`:

```python
mode: str = "dual_sim"    # Swift simulation for both arms
mode: str = "left_real"   # left real-arm teleoperation (default)
mode: str = "right_real"  # right real-arm teleoperation
```

`RealArmConfig.sdk_path` defaults to `third_party/pyAgxArm`.

## Build IK

Run this before the first run and after changing `ik_solver.pyx`:

```bash
cd analytic_control
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
python setup.py build_ext --inplace
```

## Run simulation

Set `RuntimeConfig.mode` to `"dual_sim"`, then run:

```bash
cd analytic_control
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
python run.py
```

Open Swift at `http://localhost:52000`.

## Run real-arm teleoperation

Start PICO pose publishing in one terminal from the `mm-hand` workspace:

```bash
cd read_pico
./run_ubuntu_ros.sh --vr-port 9100
```

Set `RuntimeConfig.mode` to `"left_real"` or `"right_real"`. In another
terminal, configure CAN and start control:

```bash
cd analytic_control
conda activate mmhand-ros
export PYTHONNOUSERSITE=1
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
python run.py
```

The arm first moves to the configured startup pose with the normal joint speed
limits. Press Enter in the analytic-control terminal only after that movement
finishes to begin teleoperation.

## IK overview

### Target pose

`read_pico` provides controller poses in the horizon frame. The control loop
maps each pose through the configured shoulder frame, scales its translation by
`robot_size_m / human_size_m`, and applies the fixed controller-to-flange
calibration. The result is the desired flange pose expressed in the arm base.

### Analytic 7-DoF solve

Nero has one redundant degree of freedom for a fixed flange pose. The solver
uses the arm angle `theta0` to describe where the elbow lies around the
shoulder-to-wrist line. For each sampled `theta0`, it uses the NERO modified-DH
geometry to compute the shoulder, wrist, and elbow locations, solves joints
one through four geometrically, then extracts joints five through seven from
the remaining wrist transform. This is a geometric analytic solve with an
arm-angle search, not a numerical pose optimizer.

Each arm-angle sample can produce shoulder, elbow, and wrist branches. A
candidate is discarded when any of its seven joints exceeds the configured
joint limits. Before solving, the target reach is clamped to the configured
workspace range so unreachable controller poses do not request an impossible
arm extension.

### Continuous teleoperation

The first valid target uses a global arm-angle scan to acquire a feasible
branch. Later targets search only near the previous `theta0`. Candidates are
scored by joint change from the previous two commands, forward-kinematics pose
error, arm-angle change, and proximity to joint limits. The previous branch is
kept unless another branch improves the score enough and can be reached without
a large joint jump.

Near a wrist singularity, the wrist decomposition keeps an approximate
candidate instead of dropping the pose outright; the pose-error score then
selects the best available result. The analytic IK output is subsequently
smoothed by the OneEuro joint filter and constrained by the online joint-space
motion limiter. Those filters and limits are teleoperation safety layers, not
part of the geometric IK solve.

```text
PICO pose -> calibrated flange target -> analytic IK candidates
-> continuity and limit selection -> filtering and motion limiting
```
