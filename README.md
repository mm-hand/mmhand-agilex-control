# analytic_control

Minimal Nero analytic-control runtime for PICO pose teleoperation.

The program subscribes to controller poses published by `read_pico`, maps them
through the calibrated controller/arm frames, solves Nero analytic IK, applies
OneEuro filtering and online joint-space motion limiting, then drives either
Swift simulation or one real Nero arm. It also publishes eight lightweight ROS 2
telemetry topics for current/target joints and current/target flange poses.

This is not a full ROS package. It is a normal Python entry point with an
embedded `rclpy` node, following the lightweight pattern used by `read_pico`.

## Install dependencies

Use the same ROS-enabled Python environment as `read_pico`. The repository also
has a local `.venv` that is configured for this machine.

```bash
cd /home/collin/github/mm-hand/analytic_control

# First-time setup, or after dependency changes.
.venv/bin/python -m pip install -r requirements.txt
```

If you use a RoboStack conda environment instead, keep it consistent with
`read_pico`:

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
python -m pip install -r requirements.txt
```

Do not source `/opt/ros/jazzy/setup.bash` while a RoboStack conda environment is
active. Mixing system ROS Python packages with conda packages can break imports.

## Build IK

`ik_solver.pyx` must be compiled before running:

```bash
cd /home/collin/github/mm-hand/analytic_control
.venv/bin/python setup.py build_ext --inplace
```

Generated C and shared-object files are ignored by git.

## Configure runtime

Runtime parameters live in `config.py`.

Important options:

```text
RuntimeConfig.mode             "dual_sim" | "left_real" | "right_real"
PoseInputConfig.poses_topic    /read_pose/poses
PoseInputConfig.tracked_topic  /read_pose/tracked
RealArmConfig.sdk_path         pyAgxArm SDK path
RealArmConfig.channel          SocketCAN channel, usually can0
StartupConfig.enable_real_move_j true by default
```

Current default mode is `left_real`, so `run.py` connects to the left real arm
unless you change `RuntimeConfig.mode`.

Modes:

```text
dual_sim    Run left and right arms in Swift simulation.
left_real   Subscribe to left PICO pose and send left-arm move_js commands.
right_real  Subscribe to right PICO pose and send right-arm move_js commands.
```

Simulation and real-arm teleoperation are selected by this single config value.
They are not run in parallel.

## Run simulation

Set:

```python
mode: str = "dual_sim"
```

Then run:

```bash
cd /home/collin/github/mm-hand/analytic_control
.venv/bin/python setup.py build_ext --inplace
.venv/bin/python run.py
```

Swift usually opens at:

```text
http://localhost:52000
```

Simulation publishes the same ROS telemetry topics as real-arm mode.

## Run real-arm teleoperation

Start `read_pico` first so `/read_pose/*` is available:

```bash
cd /home/collin/github/mm-hand/read_pico
./run_ubuntu_ros.sh --vr-port 9100
```

Set `RuntimeConfig.mode` in `config.py`:

```python
mode: str = "left_real"
```

or:

```python
mode: str = "right_real"
```

Bring up SocketCAN:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Run analytic control:

```bash
cd /home/collin/github/mm-hand/analytic_control
.venv/bin/python setup.py build_ext --inplace
.venv/bin/python run.py
```

Real hardware first moves to the startup pose with rate-limited `move_js`, then waits for
Enter before teleoperation starts. Teleoperation commands also use `move_js`.

## ROS 2 topics

Node: `analytic_control`

Input topics, published by `read_pico`:

```text
/read_pose/poses      geometry_msgs/PoseArray
/read_pose/tracked    std_msgs/UInt8MultiArray
```

Input conventions:

```text
poses.poses[0] = left controller grip pose
poses.poses[1] = right controller grip pose
poses.header.frame_id = "pose_input_horizon"
tracked.data = [left_valid, right_valid]
```

Output telemetry topics:

```text
/analytic_control/left/current_joints    std_msgs/Float64MultiArray
/analytic_control/left/target_joints     std_msgs/Float64MultiArray
/analytic_control/left/current_pose      geometry_msgs/PoseStamped
/analytic_control/left/target_pose       geometry_msgs/PoseStamped
/analytic_control/right/current_joints   std_msgs/Float64MultiArray
/analytic_control/right/target_joints    std_msgs/Float64MultiArray
/analytic_control/right/current_pose     geometry_msgs/PoseStamped
/analytic_control/right/target_pose      geometry_msgs/PoseStamped
```

Output conventions:

```text
current_joints.data = 7 joint angles in radians
target_joints.data  = 7 target joint angles in radians after IK filtering and motion limiting
current_pose        = FK flange pose from current joints
target_pose         = FK flange pose from target joints
pose.header.frame_id = "pose_input_horizon"
```

In `dual_sim`, both left and right telemetry are published. In `left_real` or
`right_real`, only the active side publishes non-empty telemetry.

## Coordinate conventions

`read_pico` publishes controller grip poses in the horizon frame:

```text
+X = right, +Y = up, -Z = forward
origin = head position
orientation = gravity-aligned, head yaw only
```

`analytic_control` scales and maps those controller poses through:

```text
horizon -> shoulder -> arm_base -> flange
```

Telemetry pose topics publish flange poses back in `pose_input_horizon`.

## Runtime pipeline

```text
ROS pose input
-> OneEuro pose filter
-> input speed/step limiter
-> calibrated frame transform
-> analytic IK
-> OneEuro joint-target filter
-> online joint motion limiter
-> Swift backend or pyAgxArm real-arm backend
-> ROS telemetry publishers
```

If pose input is stale or tracking is lost, the loop holds the internal motion
planner and sends no new real-arm command.

## File layout

```text
analytic_control/
+- run.py          # entry point and stdout logger
+- config.py       # runtime configuration
+- control_loop.py # teleoperation loop
+- pose_io.py      # ROS pose subscribers and telemetry publishers
+- frames.py       # controller-to-arm frame transforms
+- ik.py           # Nero IK/FK wrapper and URDF loader
+- ik_solver.pyx   # Cython analytic IK solver
+- motion.py       # online joint-space limiter
+- filters.py      # OneEuro pose/joint filters and input limiter
+- sim.py          # Swift backend
+- hardware.py     # pyAgxArm real-arm backend
+- nero_urdf/      # URDF and meshes
+- third_party/    # optional vendor SDK location
```

## Debug

Check ROS input from `read_pico`:

```bash
ros2 topic echo /read_pose/poses --once
ros2 topic echo /read_pose/tracked --once
```

Check analytic-control telemetry:

```bash
ros2 topic list | grep /analytic_control
ros2 topic echo /analytic_control/left/current_joints --once
ros2 topic echo /analytic_control/left/current_pose --once
```

Check Python imports and Cython build:

```bash
cd /home/collin/github/mm-hand/analytic_control
.venv/bin/python -m py_compile run.py control_loop.py pose_io.py ik.py hardware.py sim.py transforms.py
.venv/bin/python setup.py build_ext --inplace
```

If real-arm startup fails before connecting, check:

```bash
ls -la /sys/class/net/can0
sed -n '1p' /sys/class/net/can0/flags
.venv/bin/python -c "from config import ControlConfig; from hardware import _sdk; _sdk(ControlConfig().real); print('pyAgxArm ok')"
```
