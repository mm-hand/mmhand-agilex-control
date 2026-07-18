# analytic_control

Nero mechanical-arm analytic IK control for PICO pose teleoperation. It can
run either dual-arm Swift simulation or teleoperate one real arm.

## Before running

Prepare a Conda/RoboStack ROS environment compatible with `read_pico`. Environment
creation and dependency installation are managed separately.

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
conda activate <ros_conda_env>
export PYTHONNOUSERSITE=1
python setup.py build_ext --inplace
```

## Run simulation

Set `RuntimeConfig.mode` to `"dual_sim"`, then run:

```bash
cd analytic_control
conda activate <ros_conda_env>
export PYTHONNOUSERSITE=1
python run.py
```

Open Swift at `http://localhost:52000`.

## Run real-arm teleoperation

Start PICO pose publishing in one terminal:

```bash
cd read_pico
./run_ubuntu_ros.sh --vr-port 9100
```

Set `RuntimeConfig.mode` to `"left_real"` or `"right_real"`. In another
terminal, configure CAN and start control:

```bash
cd analytic_control
conda activate <ros_conda_env>
export PYTHONNOUSERSITE=1
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
python run.py
```

The arm first moves to the configured startup pose with the normal joint speed
limits. Press Enter in the analytic-control terminal only after that movement
finishes to begin teleoperation.
