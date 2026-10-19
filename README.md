# AgileX Nero Teleop

PICO 4 Ultra controller teleoperation for AgileX Nero arms. The repository
contains the headset receiver, the analytic IK controller, a NiceGUI console,
and the PICO APK sources.

The application runs as two Python processes:

```text
python run.py
|- parent process: NiceGUI + ArmController thread
`- pico_bridge child process: ADB, PICO TCP receiver, IPC server
```

The two processes talk over a local NDJSON TCP connection on `127.0.0.1:9101`.
The GUI and the controller share an in-memory snapshot and command queue.

## Requirements

- Linux for real Nero arms (SocketCAN `can0`, `can1`, ...).
- Windows or Linux for simulation, the GUI, and the PICO bridge.
- Python 3.12 recommended.
- PICO 4 Ultra with the supplied APK installed and USB debugging authorized.
- `adb` available in `PATH` or configured through `PicoConfig.adb_path`.
- Visual Studio Build Tools with the C++ workload on Windows, or a C compiler
  on Linux, to build the Cython IK solver.

## Install

Linux with RoboStack Jazzy:

```bash
conda env create -f environment.yml
conda activate mmhand-ros
python scripts/build_ik.sh
```

Windows simulation environment:

```powershell
conda env create -f environment-windows.yml
conda activate nero-teleop
powershell -ExecutionPolicy Bypass -File scripts/build_ik.ps1
```

Real-arm use on Linux also requires the `pyAgxArm` submodule:

```bash
git submodule update --init --recursive
```

## Run

```bash
python run.py
```

The GUI opens automatically at `http://127.0.0.1:8080`, or on the next free
port. The first tab is always `Arm Simulation`.

Useful options:

```text
--no-pico          Do not start the PICO bridge.
--demo             Use synthetic controller motion; implies --no-pico.
--headless         Run the controller without NiceGUI.
--duration SEC     Headless run time.
--port PORT        GUI port.
--vr-port PORT     PICO TCP port on the PC.
--ipc-port PORT    PICO bridge IPC port.
--no-browser       Do not open a browser.
--ros-output       Enable ROS 2 output at startup.
```

## GUI

Top bar:

- `AgileX Nero Teleop`
- `Connect Arms`
- `ROS2 Output` switch
- `STOP` on the right

Status bar: colored chips for system phase, PICO input, left/right controller
tracking, left/right arm state, and ROS 2 output.

Tabs:

- `Arm Simulation` (default): full-width 3D view of both arms. Blue is left,
  orange is right, semi-transparent green is real-arm feedback.
- `Controllers`: live position, quaternion, yaw/pitch/roll, and button states
  for both PICO controllers.

The bottom `System Log` panel is resizable. Drag the divider to change its
height. It shows control, PICO bridge, and GUI events.

## Connect arms

`Connect Arms` opens independent left and right rows. Each row has its own CAN
channel and `Connect` / `Disconnect` button. Connect what you need:

- No arm connected: pure simulation.
- Left only: left commands go to the real left arm; the right side stays
  simulated.
- Right only: right commands go to the real right arm; the left side stays
  simulated.
- Both: both sides are real; both receive the same teleoperation targets.

Left and right must use different CAN channels.

When real arms are connected, the simulation first moves to the configured
startup pose. A `Real Arm Handoff` dialog then asks you to verify that the
simulated arm bases match the real arm bases. `Confirm and Move` is disabled
until the verification checkbox is selected. Confirming moves every connected
real arm to the startup pose and enters servo teleoperation. `Cancel` does not
send motion and disconnects the real arms.

If a new arm is connected during teleoperation, the simulation returns to the
startup pose, the already-following arms hold, and the handoff dialog is shown
again for all connected arms.

`STOP` is a latched emergency stop. It stops new targets, keeps all connected
arms at their current joint positions, and disables further control. Restart
the process to resume.

## ROS 2 output

The GUI switch is off by default. When enabled, the PICO bridge publishes the
input topics and the controller publishes the arm topics. ROS 2 is imported
lazily, so a missing ROS 2 installation does not prevent simulation.

PICO input:

```text
/read_pose/poses      geometry_msgs/PoseArray
/read_pose/buttons    std_msgs/UInt32MultiArray
/read_pose/tracked    std_msgs/UInt8MultiArray
```

Nero arms:

```text
/nero/left/current_joint_angles_deg       std_msgs/Float64MultiArray
/nero/left/target_joint_angles_deg        std_msgs/Float64MultiArray
/nero/left/current_end_effector_pose      geometry_msgs/PoseStamped
/nero/left/target_end_effector_pose       geometry_msgs/PoseStamped
```

The same four topics exist under `/nero/right/`. Joint values are in degrees.
End-effector poses use `pose_input_horizon` and are the flange pose computed by
the analytic forward kinematics.

## Logs

Each run creates `logs/<timestamp>_teleop/` with `control.log` and
`telemetry.csv`. The in-memory `System Log` shows the latest control and PICO
bridge events; file logging stays available for longer sessions.

## Tests

```bash
pytest -q
```

The suite covers IK, motion limits, hardware behavior, IPC, PICO bridge helpers,
the multi-arm controller state machine, ROS topic payloads, the 3D joint chain,
and the English GUI shell. No real arm is required.

## Troubleshooting

- `ik_solver` import fails: run `scripts/build_ik.ps1` or
  `scripts/build_ik.sh`.
- PICO not found: check `adb devices`, USB debugging, and the APK package.
  The bridge retries ADB automatically.
- Real arm connection requires Linux SocketCAN. Check `ip -details link show can0`
  and the bitrate.
- Port already in use: `run.py` chooses the next free port; pass `--port`,
  `--vr-port`, or `--ipc-port` explicitly if needed.
- ROS 2 output says `UNAVAILABLE`: activate an environment with `rclpy` and
  `geometry_msgs`, then toggle it again.
