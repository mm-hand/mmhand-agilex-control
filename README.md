# analytic_control

Nero mechanical-arm analytic IK control for PICO pose teleoperation. It can
run either dual-arm Swift simulation or teleoperate one real arm.

## System prerequisites

The supported baseline is Ubuntu 24.04 x86_64. Miniforge is the recommended
Conda distribution; Mambaforge and Miniconda also work. Install the required
Ubuntu tools:

```bash
sudo apt update
sudo apt install -y git curl adb build-essential
```

`build-essential` provides the C/C++ toolchain used to compile `ik_solver.pyx`.
The real-arm mode additionally requires a Linux SocketCAN-compatible adapter
and driver that create `can0`. PICO teleoperation requires a PICO 4 Ultra with
the supplied APK installed, USB debugging enabled, and ADB authorized. Swift
simulation is viewed in a current desktop Chrome or Chromium browser.

System ROS may remain installed, but do not source
`/opt/ros/jazzy/setup.bash` in a terminal where `mmhand-ros` is active.

## Install Conda

Skip this section if `conda --version` already works. Otherwise install
Miniforge in your user directory, initialize Bash, and verify it:

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh -b -p "$HOME/miniforge3"
"$HOME/miniforge3/bin/conda" init bash
source "$HOME/.bashrc"
conda config --set auto_activate_base false
conda --version
```

Expected result: `conda --version` prints a version number. New terminals can
then use the `conda` command. The installer is provided by the
[official Miniforge project](https://github.com/conda-forge/miniforge#unix-like-platforms-macos-linux--wsl).

## Clone the workspace

`analytic_control` and `read_pico` are sibling repositories:

```text
mm-hand/
|- analytic_control/
`- read_pico/
```

Create the workspace and clone both repositories:

```bash
mkdir -p ~/mm-hand
cd ~/mm-hand
git clone --recurse-submodules https://github.com/mm-hand/analytic_control.git
git clone https://github.com/mm-hand/read_pico.git
```

## Create the ROS environment

The dependency manifest is `environment.yml`. Create the shared RoboStack Jazzy
environment once, then activate it:

```bash
cd ~/mm-hand/analytic_control
conda env create -f environment.yml
conda activate mmhand-ros
conda env config vars list
```

`environment.yml` sets `PYTHONNOUSERSITE=1` when the environment is activated,
so no manual export is needed. After pulling dependency changes, update it with:

```bash
conda env update -f environment.yml --prune
```

## Prepare and build IK

The clone command initializes the real-arm SDK submodule. To repair an existing
clone or update submodules, run:

```bash
cd ~/mm-hand/analytic_control
git submodule update --init --recursive
```

Build the Cython IK extension before the first run and after changing
`ik_solver.pyx`:

```bash
cd ~/mm-hand/analytic_control
conda activate mmhand-ros
python setup.py build_ext --inplace
```

## Verify installation

Run these checks after the IK build:

```bash
python -c "import rclpy; print('ROS Python OK')"
python -c "import roboticstoolbox as rtb; print(rtb.__version__)"
python -c "import ik_solver; print(ik_solver.__file__)"
```

Expected result: all three commands exit successfully. The second prints
`1.3.0`; the third prints a compiled `ik_solver` shared-library path.

Run the offline control regressions before connecting a real arm:

```bash
python test_ik.py
python test_motion.py
python test_hardware.py
python test_logging.py
```

Expected result: every command ends with `All ... tests passed.`. The hardware
test uses a fake SDK object; it does not open CAN or move an arm. These checks
cover IK round trips and continuity, first-command acceleration/velocity/step
limits, startup-to-servo holding, active-side routing, diagnostic rate limiting,
and telemetry serialization.

For real-arm work, verify the SocketCAN interface after configuring it:

```bash
ip -details link show can0
```

Expected result: `can0` exists, is `UP`, and reports `bitrate 1000000`.

After starting `read_pico`, verify PICO ROS input:

```bash
ros2 topic list
ros2 topic hz /read_pose/poses
ros2 topic echo /read_pose/poses --once
```

Expected result: `/read_pose/poses` is listed, its rate is non-zero while PICO
is running, and the echo prints a `geometry_msgs/PoseArray`. Stop `ros2 topic
hz` with Ctrl-C after observing the rate.

## Configure mode

Edit `RuntimeConfig.mode` in `config.py`:

```python
mode: str = "dual_sim"    # Swift simulation for both arms
mode: str = "left_real"   # left real-arm teleoperation (default)
mode: str = "right_real"  # right real-arm teleoperation
```

`RealArmConfig.sdk_path` defaults to `third_party/pyAgxArm`. Its `firmware`
setting defaults to `"auto"`; valid manual overrides are `"default"`,
`"v111"`, `"v112"`, and `"v120"`. Use an override only when automatic
detection cannot obtain the firmware version.

## Run simulation

Set `RuntimeConfig.mode` to `"dual_sim"`, then run:

```bash
cd ~/mm-hand/analytic_control
conda activate mmhand-ros
python run.py
```

Expected result:

- The terminal starts without an import error and logs `swift ready`.
- Swift is available at `http://localhost:52000`.
- Both Nero robot models appear in the browser.
- PICO controller motion updates the controller/flange axes and arm poses.

## Run real-arm teleoperation

Start PICO pose publishing in one terminal:

```bash
cd ~/mm-hand/read_pico
./run_ubuntu_ros.sh --vr-port 9100
```

Set `RuntimeConfig.mode` to `"left_real"` or `"right_real"`. In another
terminal, configure CAN and start control:

```bash
cd ~/mm-hand/analytic_control
conda activate mmhand-ros
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
python read_joints.py
python run.py
```

`read_joints.py` uses the same firmware detection and CAN connection path as
real-arm teleoperation. It must print seven live joint angles before running
`run.py`.

The arm first moves to the configured startup pose through the joint motion
limits. Press Enter only after that movement finishes. The backend then switches
to joint-servo mode while repeatedly commanding the latest measured joint
position for `servo_handoff_s`. IK continuity and the online motion planner are
reset from that same feedback. The first PICO target therefore enters through
the configured acceleration, velocity, and per-tick step limits. Missing or
stale pose input holds the last joint command.

After the control loop starts, verify telemetry:

```bash
ros2 topic list | grep /analytic_control
ros2 topic echo /analytic_control/left/current_joints --once
```

In `dual_sim`, both sides publish telemetry. In real-arm mode, the active arm
publishes non-empty current and target data.

Joint telemetry uses `sensor_msgs/JointState`; pose telemetry uses
`geometry_msgs/PoseStamped`. In real-arm mode, `current_*` headers carry the SDK
feedback timestamp, while `target_*` headers carry the host ROS publish time. In
simulation, all headers use the host ROS time.

## Run logs

File logging is enabled by `LogConfig.enabled`. Each invocation creates one
directory under `logs/`:

```text
logs/<timestamp>_<mode>/
|- control.log
`- telemetry.csv
```

`control.log` contains connection, input-state, handoff, warning, and final
summary messages. `telemetry.csv` records startup and teleoperation phases for
each active arm, including:

- raw and filtered PICO pose;
- calibrated target flange pose;
- raw IK, filtered IK, actual, and commanded seven-joint vectors;
- actual, IK, and commanded flange poses;
- real-arm current, torque, voltage, temperature, enable/error state, and arm
  status diagnostics.

Control values are sampled in the control loop. The heavier SDK diagnostics are
rate-limited by `RealArmConfig.diagnostics_hz`, which defaults to 5 Hz. CSV
data is flushed according to `LogConfig.flush_period_s`; the text log uses the
configured rotation size and backup count. Ctrl-C closes both files and prints
the final actual/commanded joints, PICO pose, and actual/IK flange poses. The
generated `logs/` directory is ignored by Git.

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
branch. Later targets search only near the previous `theta0`; global fallback
is disabled by the default runtime configuration. Candidates are scored by joint
change from the previous two commands, forward-kinematics pose error, arm-angle
change, and proximity to joint limits. The previous branch is kept unless
another branch improves the score enough and can be reached without a large
joint jump.

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
