# Linux Setup Plan

Target: Ubuntu 22.04.

This is a preparation document, not proof that the machine is already configured.

## 1. Base Tools

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl wget gnupg lsb-release
sudo apt install -y python3-pip python3-venv python3-colcon-common-extensions
```

## 2. ROS 2 Humble

Install ROS 2 Humble from the official ROS 2 documentation for Ubuntu 22.04.

Validation:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
ros2 run demo_nodes_cpp talker
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Success: the listener receives messages from the talker.

## 3. rosdep

```bash
sudo rosdep init || true
rosdep update
```

Success: `rosdep update` completes without network or source-list errors.

## 4. Workspaces

```bash
mkdir -p ~/fcr_ws/src
mkdir -p ~/tron1_ws/src
```

Recommended first layout:

```text
~/fcr_ws/src/fcr_ros2_3
~/fcr_ws/src/tron1_adapter
~/tron1_ws/src/limxsdk-lowlevel
~/tron1_ws/src/tron1-rl-deploy-ros2
~/tron1_ws/src/tron1-gazebo-ros2
~/tron1_ws/src/tron1-robot-description
~/tron1_ws/src/robot-visualization
~/tron1_ws/src/ros2-bridger
```

## 5. Fetch Code

```bash
cd ~/fcr_ws/src
git clone https://github.com/cuiangA/fcr_ros2_3.git

cd ~/tron1_ws/src
git clone https://github.com/limxdynamics/limxsdk-lowlevel.git
git clone https://github.com/limxdynamics/tron1-rl-deploy-ros2.git
git clone https://github.com/limxdynamics/tron1-gazebo-ros2.git
git clone https://github.com/limxdynamics/tron1-robot-description.git
git clone https://github.com/limxdynamics/robot-visualization.git
git clone https://github.com/limxdynamics/ros2-bridger.git
git clone https://github.com/limxdynamics/robot-joystick.git
```

TODO: Test `tron1-rl-deploy-ros2` branch `feature/humble` before committing to an Iron workflow.

## 6. FCR Dependencies

```bash
cd ~/fcr_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
pip3 install --user -r src/fcr_ros2_3/requirements.txt
```

TODO: Confirm Sony CRSDK installation path if the Sony camera is used.

TODO: Confirm Orbbec camera permissions and depth-camera model on the actual machine.

## 7. Gazebo

For the first FCR mock loop, Gazebo is not required.

For TRON1 simulation, follow the TRON1 Gazebo README matching the selected ROS 2 distribution.

TODO: Confirm whether the selected TRON1 branch uses Gazebo Classic, Gazebo Fortress, or distro-specific `ros-$ROS_DISTRO-gazebo-*` packages.

## 8. TRON1 SDK and RL Dependencies

Follow the official TRON1 RL deploy README for ONNX Runtime and model files.

TODO: Confirm exact TRON1 EDU robot type:

```text
PF_TRON1A / PF_TRON1B / SF_TRON1A / SF_TRON1B / WF_TRON1A / WF_TRON1B
```

Do not run hardware launch files until robot type, IP, controller start/stop, and e-stop behavior are confirmed.

## 9. Hardware Permissions

Likely needed:

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER
```

Log out and log back in after group changes.

For CAN:

```bash
ip link show
```

TODO: Add SocketCAN setup only after the actual CAN adapter name and bitrate are known.

For USB serial:

```bash
ls -l /dev/serial/by-id/
```

TODO: Add udev rules only after camera, serial, and CAN device IDs are known.

## 10. Build FCR

```bash
cd ~/fcr_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
source install/setup.bash
```

First success target: build completes, then run the mock/MVP launch before touching hardware.

## 11. Build TRON1

First try Humble branch if available:

```bash
cd ~/tron1_ws/src/tron1-rl-deploy-ros2
git fetch origin
git checkout feature/humble
```

Then:

```bash
cd ~/tron1_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
```

If this fails because the branch is incomplete or dependencies require Iron, stop and report the error instead of forcing packages.

## 12. Test Order

1. ROS 2 demo talker/listener.
2. FCR build.
3. FCR mock loop without camera and without chassis.
4. TRON1 SDK non-hardware examples or simulation.
5. TRON1 `/cmd_vel` simulation with manual topic publishing.
6. Adapter simulation.
7. Hardware only after the above pass.
