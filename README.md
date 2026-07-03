# simple_control

PICO controller poses to Nero dual-arm IK, visualized with Swift.

## Install

ROS 2 Jazzy must already be installed and `read_pico` must publish `/read_pico/poses` and `/read_pico/tracked`.

```bash
cd /home/starcycle/nero_control/simple_control
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m pip install --user --break-system-packages -r requirements.txt
```

## Run

Start `read_pico` first, then run:

```bash
cd /home/starcycle/nero_control/simple_control
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 run.py
```

Open the Swift URL printed by the process, typically `http://localhost:52000`.

## Files

- `run.py`: entry point
- `sim.py`: Swift visualization loop
- `ros_io.py`: ROS subscribers for `read_pico`
- `frames.py`: PICO-to-arm frame transforms
- `ik.py`: Nero IK via `roboticstoolbox`
- `config.py`: calibration and runtime parameters
- `nero_urdf/`: local URDF and meshes
