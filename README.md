# read_pico

Minimal PICO 4 Ultra controller reader.

The APK reads two controller grip poses and button states in a gravity-aligned
horizon frame (origin at the head, yaw-only, no pitch/roll) derived from OpenXR
`LOCAL` space, then sends them to the PC over USB with `adb reverse`. The PC
side can publish ROS 2 topics or show a browser visualizer.

This project does not use ALVR.

## Prebuilt APK

```text
apk/generated_apk/pico_head_relative_pose.apk
```

Use this first. You only need the APK build toolchain to rebuild the headset app.

## Install dependencies

### PC-side tools (Ubuntu)

```bash
sudo apt install -y adb python3 python3-aiohttp
```

### ROS 2 (optional)

```bash
source <(wget -qO- http://fishros.com/install)
sudo apt install -y ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-std-msgs
# Humble: replace jazzy with humble.
```

### APK build toolchain (Ubuntu only)

```bash
sudo apt install -y openjdk-17-jdk
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
cargo install cargo-apk
sdkmanager "platform-tools" "platforms;android-31" "build-tools;32.0.0" "ndk;25.1.8937393"
```

`apk/build.sh` locates the toolchain via env vars (defaults shown):

```text
JAVA_HOME        /usr/lib/jvm/java-17-openjdk-amd64
ANDROID_HOME     $HOME/Android/Sdk
ANDROID_NDK_HOME $ANDROID_HOME/ndk/25.1.8937393
```

## Run

Build the APK, only if modifying the headset app:

```bash
cd apk && ./build.sh
```

Install the APK:

```bash
adb install -r apk/generated_apk/pico_head_relative_pose.apk
```

ROS communication:

```bash
./run_ubuntu_ros.sh --vr-port 9100
# Humble:
ROS_DISTRO=humble ./run_ubuntu_ros.sh --vr-port 9100
```

Visual-only, no ROS:

```bash
python pc/run.py --vr-port 9100 --visualize-port 8765
```

Receiver commands run `adb reverse tcp:29761 tcp:<vr-port>` and launch the APK.

## ROS 2 topics

Node: `read_pico`

```text
/read_pico/poses      geometry_msgs/PoseArray
/read_pico/buttons    std_msgs/UInt32MultiArray
/read_pico/tracked    std_msgs/UInt8MultiArray
```

Conventions:

```text
poses.poses[0] = left, poses.poses[1] = right
poses.header.frame_id = "pico_horizon"
buttons.data = [left_buttons, right_buttons]
tracked.data = [left_valid, right_valid]
```

## Data format

Each TCP frame is one 19-column CSV line:

```text
timestamp,left_valid,lx,ly,lz,lqx,lqy,lqz,lqw,right_valid,rx,ry,rz,rqx,rqy,rqz,rqw,left_buttons,right_buttons
```

Horizon frame (derived from OpenXR `LOCAL`):

```text
+X = right, +Y = up, -Z = forward
origin = head position (follows the head, including height)
orientation = gravity-aligned, head yaw only (no pitch/roll)
ground plane = XZ
```

Controller grip frame (per controller):

```text
origin = grip point of the controller
+X = right, +Y = up, -Z = forward (when held naturally)
pose = controller origin expressed in the horizon frame
```

Button masks:

```text
left:  bit0 X, bit1 Y, bit2 Menu, bit3 Trigger, bit4 Grip, bit5 Thumbstick
right: bit0 A, bit1 B, bit2 Trigger, bit3 Grip, bit4 Thumbstick
```

## File layout

```text
read_pico/
+- run_ubuntu_ros.sh
+- pc/
|  +- run.py       # receiver, ADB reverse, ROS or web
|  `- view.html    # Canvas visualizer
+- apk/
|  +- build.sh     # Ubuntu APK build
|  +- generated_apk/pico_head_relative_pose.apk
|  `- src/
|     +- lib.rs     # OpenXR loop, yaw-only horizon frame
|     +- net.rs     # TCP non-blocking send
|     +- display.rs # headset status image
|     `- egl.rs     # EGL/OpenGL setup
```

## Debug

PC side: `python pc/run.py ... 2>&1 | tee debug.log`

PICO side: `adb logcat -s PicoPose:V`
