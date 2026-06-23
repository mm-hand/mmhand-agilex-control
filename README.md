# read_pico

Minimal PICO 4 Ultra controller reader.

The APK reads the left and right controller grip poses, plus button states, in
the headset-relative OpenXR `VIEW` space. It sends the data to the PC over USB
through `adb reverse`. The PC receiver can publish ROS 2 topics, record CSV, and
optionally open a tiny browser visualizer.

This project does not use ALVR.

## Install dependencies

### Windows

For receiving data without ROS:

```text
Python
ADB
```

For browser visualization:

```text
aiohttp
```

For native ROS 2 publishing on Windows, this repository currently expects the
prepared Jazzy/Pixi layout:

```text
C:\pixi_ws
+- pixi.toml
`- ros2-windows\
```

`run_windows_ros.cmd` uses Pixi's Python 3.12.3, loads
`C:\pixi_ws\ros2-windows\local_setup.bat`, and then runs `pc\run.py`.

The system Python is not used for ROS 2 on Windows.

### Ubuntu

For receiving data without ROS:

```bash
sudo apt install adb python3
```

For browser visualization:

```bash
sudo apt install python3-aiohttp
```

For ROS 2 publishing, install ROS 2 Jazzy or Humble and the standard Python
message packages:

```bash
sudo apt install \
  ros-$ROS_DISTRO-rclpy \
  ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-std-msgs
```

The Ubuntu receiver does not need Pixi.

### APK build toolchain

APK compilation is currently set up for Windows through `apk/build.ps1`.

The script uses the project-local Android/Rust/JDK toolchain expected under:

```text
read_pico/toolchains/
```

Ubuntu can run the PC receiver, but this repository does not yet provide a
Linux APK build script. A future `apk/build.sh` would need the same pieces:

```text
JDK 17
Rust
Android SDK
Android NDK
Android build-tools
cargo target aarch64-linux-android
```

## Run

### Build the APK

Windows:

```powershell
cd apk
.\build.ps1
```

The built APK is written to:

```text
apk/generated_apk/pico_head_relative_pose.apk
```

### Install the APK

```bash
adb install -r apk/generated_apk/pico_head_relative_pose.apk
```

Start the app on the PICO headset after installation.

### Start without visualization

With ROS 2 enabled:

```bash
python pc/run.py --vr-port 9100
```

Without ROS 2:

```bash
python pc/run.py --vr-port 9100 --no-ros
```

`--vr-port` is the PC TCP port receiving live PICO data.

Internally the receiver creates:

```text
PICO tcp:29761 -> adb reverse -> PC tcp:<vr-port>
```

### Start with browser visualization

```bash
python pc/run.py --vr-port 9100 --visualize --visualize-port 8765
```

`--visualize-port` is only the local browser HTTP/WebSocket port.

The two ports must both be within `1..65535`, and they must be different when
visualization is enabled.

### Windows ROS 2 shortcut

```bat
run_windows_ros.cmd --vr-port 9100
run_windows_ros.cmd --vr-port 9100 --visualize --visualize-port 8765
```

### Ubuntu ROS 2 shortcut

Jazzy is the default:

```bash
./run_ubuntu_ros.sh --vr-port 9100
```

For Humble:

```bash
ROS_DISTRO=humble ./run_ubuntu_ros.sh --vr-port 9100
```

### Record CSV

```bash
python pc/run.py --vr-port 9100 --record
```

CSV files are written under:

```text
recordings/
```

### ROS 2 topics

The node name is:

```text
read_pico
```

Published topics:

```text
/read_pico/poses      geometry_msgs/PoseArray
/read_pico/buttons    std_msgs/UInt32MultiArray
/read_pico/tracked    std_msgs/UInt8MultiArray
```

Conventions:

```text
poses.poses[0] = left
poses.poses[1] = right
poses.header.frame_id = "pico_head"

buttons.data = [left_buttons, right_buttons]
tracked.data = [left_valid, right_valid]
```

The topics use sensor-data QoS. `PoseArray` timestamps are assigned on the PC
when the frame is received.

If a controller is not tracked, its pose is zero position with identity
orientation. Consumers should trust `tracked`, not the zero pose.

## Data format

Each TCP frame is one CSV line with 19 columns:

```text
timestamp,left_valid,lx,ly,lz,lqx,lqy,lqz,lqw,right_valid,rx,ry,rz,rqx,rqy,rqz,rqw,left_buttons,right_buttons
```

Positions are in metres. Quaternions use `(x,y,z,w)`.

The coordinate frame is OpenXR `VIEW` space:

```text
+X = right
+Y = up
-Z = forward
origin = headset view
```

Button bitmasks:

```text
left:
  bit 0 = X
  bit 1 = Y
  bit 2 = Menu
  bit 3 = Trigger
  bit 4 = Grip
  bit 5 = Thumbstick

right:
  bit 0 = A
  bit 1 = B
  bit 2 = Trigger
  bit 3 = Grip
  bit 4 = Thumbstick
```

## File layout

```text
read_pico/
+- run_windows_ros.cmd
+- run_ubuntu_ros.sh
+- README.md
+- pc/
|  +- run.py
|  `- view.html
+- apk/
|  +- AndroidManifest.xml
|  +- build.ps1
|  +- Cargo.toml
|  +- src/
|  |  +- lib.rs
|  |  +- net.rs
|  |  +- display.rs
|  |  `- egl.rs
|  +- assets/
|  |  +- connected.rgba
|  |  `- disconnected.rgba
|  +- generated_apk/
|  |  `- pico_head_relative_pose.apk
|  +- runtime_libs/
|  `- java/com/mmhand/pico_head_relative_pose/MainActivity.java
`- recordings/
```

Important files:

```text
pc/run.py        PC receiver, ADB reverse, ROS publisher, optional CSV/web.
pc/view.html     Small Canvas visualizer.

apk/src/lib.rs   OpenXR session and per-frame pose/button readout.
apk/src/net.rs   TCP reconnect and CSV sending.
apk/src/display.rs
                 Headset connected/disconnected status image.
apk/src/egl.rs   EGL/OpenGL setup.
```

## Code logic

Runtime path:

```text
PICO OpenXR app
  -> read left/right grip pose
  -> read left/right buttons
  -> format one 19-column CSV line
  -> TCP connect to 127.0.0.1:29761 on the headset
  -> adb reverse maps that to PC --vr-port
  -> pc/run.py receives and validates the frame
  -> optional ROS 2 publish
  -> optional browser visualization
  -> optional CSV recording
```

The APK treats TCP write success as `CONNECTED`; failed write or no socket means
`DISCONNECTED`. The headset display shows only that state.

The browser visualizer is deliberately separate from the TCP receive path. It
uses the latest received frame and draws with `requestAnimationFrame`, so slow
browser rendering does not block pose receiving.
