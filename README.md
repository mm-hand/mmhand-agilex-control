# analytic_control

PICO controller poses to Nero dual-arm IK, visualized with Swift.

## Install

ROS 2 Jazzy must already be installed and `read_pico` must publish `/read_pico/poses` and `/read_pico/tracked`.

```bash
cd /home/starcycle/nero_control/analytic_control
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m pip install --user --break-system-packages -r requirements.txt
```

## Build (Cython)

The IK solver (`ik_solver.pyx`) must be compiled with Cython before running:

```bash
cd /home/starcycle/nero_control/analytic_control
/usr/bin/python3 setup.py build_ext --inplace
```

This produces a compiled `.so` that is imported automatically. Rebuild after modifying `ik_solver.pyx`.

## Run

Start `read_pico` first, then run:

```bash
cd /home/starcycle/nero_control/analytic_control
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 run.py
```

Open the Swift URL printed by the process, typically `http://localhost:52000`.

## Test

```bash
cd /home/starcycle/nero_control/analytic_control
/usr/bin/python3 test_ik.py
```

## Files

- `run.py`: entry point
- `sim.py`: Swift visualization loop
- `ros_io.py`: ROS subscribers for `read_pico`
- `frames.py`: PICO-to-arm frame transforms
- `ik.py`: Nero IK wrapper (analytic arm-angle IK + Swift visualization robot)
- `ik_solver.pyx`: analytic arm-angle parameterized IK solver (compiled with Cython)
- `setup.py`: Cython build configuration
- `config.py`: calibration and runtime parameters
- `transforms.py`: quaternion/RPY/4x4 transform math utilities
- `test_ik.py`: IK verification tests
- `nero_urdf/`: local URDF and meshes
