# read_pico

Minimal PICO 4 Ultra controller reader.

The APK reads two controller grip poses and button states in headset-relative
OpenXR `VIEW` space, then sends them to the PC over USB with `adb reverse`.
The PC side can publish ROS 2 topics, record CSV, and optionally show a browser
visualizer.

This project does not use ALVR.

## Prebuilt APK

The repository already contains a compiled APK:

```text
apk/generated_apk/pico_head_relative_pose.apk
```

Use this first. You only need the APK build toolchain if you want to modify and
rebuild the headset app.

## Install dependencies

### Windows, no ROS 2

Install Python:

```powershell
winget install -e --id Python.Python.3.12
```

Install ADB platform-tools:

```powershell
mkdir C:\Android -Force
curl.exe -L --fail -o C:\Android\platform-tools.zip https://dl.google.com/android/repository/platform-tools-latest-windows.zip
Expand-Archive C:\Android\platform-tools.zip C:\Android -Force
$env:Path = "C:\Android\platform-tools;$env:Path"
```

Install the optional browser dependency:

```powershell
python -m pip install aiohttp
```

### Windows, native ROS 2 Jazzy

Install Pixi:

```powershell
powershell -ExecutionPolicy Bypass -Command "iwr -useb https://pixi.sh/install.ps1 | iex"
$env:Path = "$env:USERPROFILE\.pixi\bin;$env:Path"
```

Create the ROS 2 workspace expected by `run_windows_ros.cmd`:

```powershell
mkdir C:\pixi_ws -Force
cd C:\pixi_ws
curl.exe -L --fail -o pixi.toml https://raw.githubusercontent.com/ros2/ros2/jazzy/pixi.toml
& "$env:USERPROFILE\.pixi\bin\pixi.exe" install
```

Download and extract the tested ROS 2 Jazzy Windows binary:

```powershell
curl.exe -L --fail -o ros2-jazzy-20260128-windows-release-amd64.zip https://github.com/ros2/ros2/releases/download/release-jazzy-20260128/ros2-jazzy-20260128-windows-release-amd64.zip
if ((Get-FileHash .\ros2-jazzy-20260128-windows-release-amd64.zip -Algorithm SHA256).Hash.ToLower() -ne "97684cf478af8947429f089de7b55d3037e006f05f3faafb21082a47956f5a9a") { throw "bad ROS zip hash" }
Expand-Archive .\ros2-jazzy-20260128-windows-release-amd64.zip C:\pixi_ws -Force
& "$env:USERPROFILE\.pixi\bin\pixi.exe" add aiohttp
```

Quick check:

```powershell
& "$env:USERPROFILE\.pixi\bin\pixi.exe" run cmd /d /s /c "call C:\pixi_ws\ros2-windows\local_setup.bat && python -c ""import rclpy; import geometry_msgs; import std_msgs; print('ROS ok')"""
```

### Ubuntu

Install PC-side tools:

```bash
sudo apt update
sudo apt install -y adb python3 python3-aiohttp wget
```

Install ROS 2 with FishROS:

```bash
source <(wget -qO- http://fishros.com/install)
```

In the menu, choose ROS/ROS2 installation. Use Jazzy on Ubuntu 24.04 and Humble
on Ubuntu 22.04.

After installation, make sure these message packages exist.

Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install -y ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-std-msgs
```

Humble:

```bash
source /opt/ros/humble/setup.bash
sudo apt install -y ros-humble-rclpy ros-humble-geometry-msgs ros-humble-std-msgs
```

FishROS is convenient, especially on China mainland networks. If you need a
fully deterministic install path, use the official ROS 2 Ubuntu apt guide
instead.

### APK build toolchain

APK rebuilding is currently Windows-only in this repository.

`apk/build.ps1` expects an ignored local toolchain directory:

```text
toolchains/
```

Expected pieces:

```text
JDK 17.0.19+10
Rust + cargo-apk
Android SDK platform android-31
Android build-tools 32.0.0
Android NDK 25.1.8937393
Android platform-tools
```

No Linux APK build script is provided yet.

## Run

Install the APK:

```bash
adb install -r apk/generated_apk/pico_head_relative_pose.apk
```

Start the app on the PICO headset.

Build the APK on the prepared Windows workspace only if needed:

```powershell
cd apk
.\build.ps1
```

Start the PC receiver without visualization:

```bash
python pc/run.py --vr-port 9100
```

Start without ROS 2:

```bash
python pc/run.py --vr-port 9100 --no-ros
```

Start with browser visualization:

```bash
python pc/run.py --vr-port 9100 --visualize --visualize-port 8765
```

Windows ROS shortcut:

```bat
run_windows_ros.cmd --vr-port 9100
run_windows_ros.cmd --vr-port 9100 --visualize --visualize-port 8765
```

Ubuntu ROS shortcut:

```bash
./run_ubuntu_ros.sh --vr-port 9100
ROS_DISTRO=humble ./run_ubuntu_ros.sh --vr-port 9100
```

Record CSV:

```bash
python pc/run.py --vr-port 9100 --record
```

Port meanings:

```text
--vr-port         PC TCP port receiving PICO data.
--visualize-port  Local browser HTTP/WebSocket port.
```

The receiver creates:

```text
PICO tcp:29761 -> adb reverse -> PC tcp:<vr-port>
```

## ROS 2 topics

Node:

```text
read_pico
```

Topics:

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

The topics use sensor-data QoS. Pose timestamps are assigned on the PC when a
frame is received.

## Data format

Each TCP frame is one 19-column CSV line:

```text
timestamp,left_valid,lx,ly,lz,lqx,lqy,lqz,lqw,right_valid,rx,ry,rz,rqx,rqy,rqz,rqw,left_buttons,right_buttons
```

OpenXR `VIEW` frame:

```text
+X = right
+Y = up
-Z = forward
origin = headset view
```

Button masks:

```text
left:  bit0 X, bit1 Y, bit2 Menu, bit3 Trigger, bit4 Grip, bit5 Thumbstick
right: bit0 A, bit1 B,            bit2 Trigger, bit3 Grip, bit4 Thumbstick
```

## File layout

```text
read_pico/
+- run_windows_ros.cmd
+- run_ubuntu_ros.sh
+- pc/
|  +- run.py       # receiver, ADB reverse, ROS, CSV, optional web
|  `- view.html    # tiny Canvas visualizer
+- apk/
|  +- build.ps1    # Windows APK build
|  +- generated_apk/pico_head_relative_pose.apk
|  `- src/
|     +- lib.rs     # OpenXR loop
|     +- net.rs     # TCP reconnect/send
|     +- display.rs # headset status image
|     `- egl.rs     # EGL/OpenGL setup
`- recordings/
```

## Code logic

```text
PICO OpenXR app
  -> read left/right pose and buttons
  -> send one CSV line to headset tcp:29761
  -> adb reverse forwards it to PC --vr-port
  -> pc/run.py validates the frame
  -> publish ROS 2 topics, optionally visualize or record
```

The browser uses the latest received frame and draws with
`requestAnimationFrame`, so rendering does not block TCP receiving.

## Git ignore policy

Tracked:

```text
apk/generated_apk/pico_head_relative_pose.apk
```

Ignored:

```text
toolchains/
apk/target/
recordings/*.csv
__pycache__/
*.pyc
```
