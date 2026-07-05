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

Use the system Python for ROS 2. This matters if Conda or another Python is
active: ROS Jazzy Debian packages are built for Ubuntu's `/usr/bin/python3`,
not a Conda `python3`.

```bash
sudo apt install -y adb python3-aiohttp
source <(wget -qO- http://fishros.com/install)
sudo apt update
sudo apt install -y ros-jazzy-rclpy ros-jazzy-geometry-msgs ros-jazzy-std-msgs
# If you use Humble: replace jazzy with humble.
```

If ROS 2 was already installed before running the FishROS installer, align the
installed Jazzy packages to the same apt repository batch. Mixed old/new ROS
packages can break `rclpy.create_node()` before this project reaches ADB.

```bash
sudo apt install -y --only-upgrade $(dpkg -l 'ros-jazzy-*' | awk '/^ii/ {print $2}')
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c 'import rclpy; rclpy.init(args=[]); n=rclpy.create_node("probe"); print("ok"); n.destroy_node(); rclpy.shutdown()'
```

### APK build toolchain (Ubuntu only, if you want to modify apk)

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
/usr/bin/python3 pc/run.py --vr-port 9100 --visualize-port 8765
```

Receiver commands run `adb reverse tcp:29761 tcp:<vr-port>` and launch the APK.

## ROS 2 topics

Node: `read_pose`

```text
/read_pose/poses      geometry_msgs/PoseArray
/read_pose/buttons    std_msgs/UInt32MultiArray
/read_pose/tracked    std_msgs/UInt8MultiArray
```

Conventions:

```text
poses.poses[0] = left, poses.poses[1] = right
poses.header.frame_id = "pose_input_horizon"
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

PC side: `/usr/bin/python3 pc/run.py ... 2>&1 | tee debug.log`

PICO side: `adb logcat -s PicoPose:V`
