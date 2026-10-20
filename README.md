# AgileX Nero Teleop

PICO 4 Ultra controller teleoperation for AgileX Nero arms.

The repository contains:

- a PICO headset receiver with ADB setup
- a two-process teleoperation controller
- analytic seven-axis inverse kinematics
- a NiceGUI web console
- a URDF/STL-based dual-arm simulation
- optional ROS 2 publishing
- the PICO APK sources and prebuilt APK

## System Architecture

The application runs as two Python processes:

```text
python run.py
├── parent process
│   ├── NiceGUI web console
│   ├── ArmController thread
│   ├── simulation state
│   ├── real-arm backends
│   └── optional ROS 2 publisher
└── pico_bridge child process
    ├── ADB reverse and APK launch
    ├── PICO TCP receiver
    └── local IPC server
```

Data flow:

```text
PICO APK
  │ TCP 29761
  ▼
adb reverse
  ▼
pico_bridge
  │ NDJSON over 127.0.0.1:9101
  ▼
IpcPoseSource
  ▼
ArmController ──► simulation q ──► NiceGUI 3D scene
      │
      ├──► RealArmBackend (left/right, SocketCAN)
      └──► RosOutput (/nero/left/*, /nero/right/*)
```

The GUI and controller share an immutable snapshot and a command queue. The
PICO bridge is a separate process so headset reconnects and ADB failures do not
block the UI or the controller.

## Requirements

- Python 3.12 recommended
- Linux for real Nero arms with SocketCAN
- Windows or Linux for simulation, GUI, and PICO bridge
- PICO 4 Ultra with the supplied APK installed
- USB debugging enabled and authorized
- `adb` available in `PATH` or configured in `PicoConfig.adb_path`
- Cython and a C/C++ compiler
  - Visual Studio Build Tools with the C++ workload on Windows
  - a C compiler such as GCC on Linux

## Installation

Use an existing conda environment. Do not create a new environment. The
following examples assume the existing environment is named `mmhand`; replace it
with the environment you already use.

The environment must already provide Python 3.12 and, for ROS 2 output, the
ROS 2 Python packages (`rclpy`, `geometry_msgs`, `std_msgs`). If those packages
are missing, install them into the same environment with conda; do not clone the
environment.

### Install Python dependencies

```bash
conda activate mmhand
pip install -r requirements.txt
```

`requirements.txt` installs the project dependencies into the active conda
environment. It does not create an environment.

### Build the IK extension

The IK solver is the only compiled part of the project. Cython generates C from
`ik_solver.pyx`, then the platform compiler builds a native extension.

Windows and Linux use the same commands. The only difference is the compiler
environment:

- Windows: open **x64 Native Tools Command Prompt for VS 2022** so `cl.exe` is
  on `PATH`.
- Linux: a working GCC toolchain.

```bash
conda activate mmhand
cd <repository>
python setup.py build_ext --inplace
```

Expected build artifacts:

```text
ik_solver.c
ik_solver.cp312-win_amd64.pyd   # Windows
ik_solver.cpython-312-*.so      # Linux
```

The generated files are ignored by Git. Rebuild after changing `ik_solver.pyx`.
If Cython is missing, run `pip install -r requirements.txt` in the same
environment.

For real-arm use on Linux, initialize the SDK submodule:

```bash
git submodule update --init --recursive
```

Windows supports simulation, GUI, IPC, and ROS 2 output. Real Nero arms require
Linux SocketCAN.

### PICO APK

Install the prebuilt APK:

```bash
adb install -r apk/generated_apk/pico_head_relative_pose.apk
```

The `pico_bridge` process runs `adb reverse` and launches the APK automatically.

## Running

```bash
python run.py
```

The GUI opens at `http://127.0.0.1:8080`, or on the next free port.

Options:

```text
--no-pico          do not start the PICO bridge
--demo             use synthetic controller motion; implies --no-pico
--headless         run the controller without NiceGUI
--duration SEC     headless run time
--port PORT        GUI port
--vr-port PORT     PICO TCP port on the PC
--ipc-port PORT    PICO bridge IPC port
--no-browser       do not open a browser
```

Examples:

```bash
python run.py
python run.py --no-pico --demo
python run.py --no-pico --headless --duration 5
```

## GUI

Top bar:

- `AgileX Nero Teleop`
- `Connect Arms`
- `ROS 2 Output` switch
- `STOP`

The status bar contains colored chips for:

- system phase
- PICO input
- left/right controller tracking
- left/right arm state
- ROS 2 output

Tabs:

- `Arm Simulation` (first tab and default): full-width 3D view of both arms
- `Controllers`: live pose and button state for both PICO controllers
- `Settings`: startup pose and arm base frame overrides

The bottom `System Log` is resizable. Drag the divider to change its height.

## Settings

`Settings` edits a runtime override file:

```text
settings.json
```

It does not rewrite `config.py`. `config.py` remains the source of default
values. The file is ignored by Git.

Editable fields:

- left and right startup pose, seven joint angles in degrees
- horizon-to-shoulder translation for each arm
- shoulder-to-arm-base translation and RPY for each arm

Buttons:

- `Save`: validate and write `settings.json`
- `Restore Defaults`: remove `settings.json` and restore the displayed defaults

Settings are applied at the next `python run.py`. The page shows
`Restart to apply.` after a save. This avoids changing arm-base coordinates
during an active teleoperation session.

If `settings.json` is missing or invalid, the application logs a warning and
uses the `config.py` defaults.

## Operating the Arms

### Connecting arms

`Connect Arms` opens independent left and right rows. Each row has its own CAN
channel and `Connect` / `Disconnect` button.

- No arm connected: pure simulation
- Left only: left arm uses hardware, right arm remains simulated
- Right only: right arm uses hardware, left arm remains simulated
- Both: both arms use hardware

Left and right must use different CAN channels.

### Handoff

After connecting real arms, the simulation moves to the configured startup pose.
A `Real Arm Handoff` dialog then asks the operator to verify that each simulated
arm base matches its real arm base. `Confirm and Move` is disabled until the
verification checkbox is selected.

Confirming moves every connected real arm to the startup pose and enters servo
teleoperation. Cancelling sends no motion and disconnects the real arms.

If a new arm connects during teleoperation, the simulation returns to the
startup pose, the following arms hold, and the handoff dialog is shown again for
all connected arms.

### STOP

`STOP` is a latched emergency stop. It stops new targets, keeps all connected
arms at their current joint positions, and disables further control. Restart the
process to resume.

### Logging

Runtime messages go to the console, the GUI `System Log`, and a rotating file:

```text
logs/control.log
```

The file log is enabled by `LogConfig.enabled`. The GUI log is resizable and
shows control and PICO bridge events.

### ROS 2 output

The switch is off by default. When enabled, the PICO bridge publishes the input
topics and the controller publishes the arm topics:

```text
/read_pose/poses
/read_pose/buttons
/read_pose/tracked

/nero/left/current_joint_angles_deg
/nero/left/target_joint_angles_deg
/nero/left/current_end_effector_pose
/nero/left/target_end_effector_pose

/nero/right/current_joint_angles_deg
/nero/right/target_joint_angles_deg
/nero/right/current_end_effector_pose
/nero/right/target_end_effector_pose
```

Joint values are in degrees. End-effector poses are `geometry_msgs/PoseStamped`
in `pose_input_horizon`.

## How It Works

### PICO input

The APK sends one CSV line per controller sample:

```text
timestamp,left_valid,lx,ly,lz,lqx,lqy,lqz,lqw,
right_valid,rx,ry,rz,rqx,rqy,rqz,rqw,left_buttons,right_buttons
```

Poses are expressed in a gravity-aligned horizon frame:

```text
+X = right
+Y = up
-Z = forward
origin = head position
orientation = yaw only
```

### IPC

The PICO bridge and controller communicate over a local TCP connection:

- NDJSON, one message per line
- `pico_frame` for controller samples
- `pico_status` for ADB and frame-rate state
- `command` / `command_ack` for ROS output and shutdown
- latest-frame semantics for high-rate input

### Analytic IK

`ik_solver.pyx` solves a seven-axis Nero arm from an end-effector pose:

1. Sample the redundant arm angle `theta0`.
2. Solve joints 1-4 geometrically from the elbow position.
3. Extract joints 5-7 from the remaining wrist transform.
4. Discard candidates outside joint limits.
5. Score candidates by joint continuity, pose error, arm-angle change, and
   joint-limit proximity.

`ik.py` converts the flange target to the link-7 frame using the URDF
`gripper_flange_joint` origin.

### URDF mesh simulation

The simulation uses the real Nero URDF and STL meshes:

```text
nero_urdf/nero.urdf
nero_urdf/meshes/*.stl
```

`ui/scene.py` parses the URDF joint tree:

- joint parent and child links
- joint origin `xyz` and `rpy` (URDF `rpy` is in radians)
- joint axis
- visual mesh and visual origin

Each joint is represented by a fixed origin group and a rotating group. Only
the seven joint rotations are updated during teleoperation. The STL meshes are
served by NiceGUI at `/nero_meshes/*.stl`.

The left arm is blue, the right arm is orange, and real-arm feedback is drawn as
a semi-transparent green ghost.

### Filtering and motion limits

The controller applies:

- OneEuro pose filtering
- input position and rotation limiting
- IK output filtering
- joint velocity, acceleration, and per-tick step limits
- joint-limit margin clipping

### Simulation and real arms

Both sides always maintain a simulated joint state. A connected side also sends
the same target to its real arm. When no PICO pose is available, that side holds
its current target. During handoff, real arms move to the startup pose, enter
servo hold, and then follow the teleoperation targets.

## Code Structure and Line Counts

Line counts are total file lines, including blank lines and comments.

```text
analytic_control/
├── run.py                              183
├── config.py                           160
├── control_loop.py                     507
├── controller_state.py                  67
├── settings_store.py                    92
├── ipc.py                              238
├── pose_source.py                      147
├── ros_output.py                       131
├── hardware.py                         404
├── ik.py                                73
├── ik_solver.pyx                       605
├── filters.py                          149
├── frames.py                            85
├── motion.py                            81
├── transforms.py                        67
├── joint_feedback.py                    30
├── logging_utils.py                     73
├── read_joints.py                       21
├── setup.py                             21
│
├── pico_bridge/
│   ├── __init__.py                       0
│   ├── __main__.py                      65
│   ├── bridge.py                       191
│   └── ros_output.py                    85
│
├── ui/
│   ├── __init__.py                       0
│   ├── app.py                          279
│   ├── controllers.py                   66
│   ├── scene.py                        288
│   └── settings.py                      88
│
├── test/
│   ├── __init__.py                       0
│   ├── test_controller.py              164
│   ├── test_hardware.py                349
│   ├── test_ipc.py                      65
│   ├── test_kinematics.py              131
│   ├── test_logging.py                  42
│   ├── test_pico_bridge.py              47
│   ├── test_ros_output.py               76
│   ├── test_settings.py                 63
│   └── test_ui.py                      135
│
├── requirements.txt
├── pytest.ini                             4
├── settings.json                          # generated, ignored by Git
└── README.md
```

Source code totals, excluding this README, requirements, and pytest.ini:

| Group | Files | Lines |
|---|---:|---:|
| Core control and kinematics | 17 | 3,092 |
| PICO bridge | 4 | 341 |
| NiceGUI | 5 | 721 |
| Tests | 10 | 1,072 |
| Setup | 1 | 21 |
| Hardware debug tool | 1 | 21 |
| **Total** | **38** | **5,268** |

`nero_urdf/meshes/*.stl` are runtime assets, not code. `apk/`, `build/`,
generated `.pyd`/`.c` files, and `logs/` are excluded from the code count.

## Testing

```bash
pytest -q
```

Tests live in `test/`. `pytest.ini` sets:

```ini
testpaths = test
pythonpath = .
asyncio_mode = auto
```

Coverage includes:

- analytic IK and motion limits
- URDF chain versus analytic forward kinematics
- real-arm feedback and motion behavior
- two-process IPC
- PICO bridge helpers
- ROS 2 topic payloads
- dual-arm handoff and STOP behavior
- English UI, status colors, System Log, and handoff dialog
- file and in-memory logging

## Troubleshooting

- `ik_solver` import fails: build it with
  `python setup.py build_ext --inplace`.
- `ModuleNotFoundError: No module named 'Cython'`: activate the existing
  environment and run `pip install -r requirements.txt`, then build again.
- Windows build cannot find `cl.exe`: use an x64 Native Tools Command Prompt for
  VS 2022.
- PICO not found: check `adb devices`, USB debugging, and the APK package. The
  bridge retries ADB automatically.
- Real arm connection fails: verify the SocketCAN channel and bitrate.
- Port already in use: `run.py` selects the next free port; pass `--port`,
  `--vr-port`, or `--ipc-port` explicitly if needed.
- ROS 2 output says `UNAVAILABLE`: the active environment needs `rclpy`,
  `geometry_msgs`, and `std_msgs`. Install them into the same environment with
  conda, then toggle again:
  ```bash
  conda install -c conda-forge -c robostack-jazzy \
    ros-jazzy-ros-base ros-jazzy-geometry-msgs ros-jazzy-std-msgs
  ```
- NumPy matrix operations crash with a DLL or OpenBLAS error: reinstall NumPy
  into the same environment:
  ```bash
  python -m pip install --force-reinstall --no-cache-dir "numpy>=1.26"
  ```
