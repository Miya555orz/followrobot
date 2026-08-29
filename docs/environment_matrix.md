# Environment Matrix

Date: 2026-08-29

## FCR ROS 2 Project

```makefile
fcr_ros2_3
OS: Ubuntu 22.04 is the expected practical target, confirmed by the Humble CI workflow.
ROS2: ROS 2 Humble.
Python: Python 3 on Ubuntu 22.04, typically 3.10; exact patch version not pinned.
Gazebo: Optional for full simulation. MVP mock tests do not require Gazebo.
Other dependencies: colcon, rosdep, OpenCV, cv_bridge, image_transport, tf2, Eigen, ONNX Runtime vendor, optional TensorRT/CUDA, Orbbec camera packages, Sony CRSDK, foxglove_bridge, curl, PortAudio, nlohmann-json.
```

## TRON1 Official Stack

```makefile
TRON1
OS: Official ROS 2 deployment docs point to Ubuntu 22.04.
ROS2: Official main deployment path uses ROS 2 Iron; tron1-rl-deploy-ros2 also has feature/humble and feature/foxy branches that must be tested.
Python: Python 3 on Ubuntu 22.04, typically 3.10; exact patch version not pinned.
Gazebo: Official ROS 2 Gazebo stack targets Ubuntu 22.04 with Gazebo/ros2_control packages for Iron.
SDK: limxsdk-lowlevel, C++11/Python SDK exposing low-level RobotCmd/RobotState/IMU APIs.
Other dependencies: ONNX Runtime 1.10.0 in official RL deploy docs, robot description, robot visualization, ros2_control, controller_manager, Gazebo integration, joystick tooling.
```

## Conflict Judgment

Ubuntu 22.04 should not be replaced. The real conflict is ROS 2 distribution choice:

- FCR is built around Humble.
- TRON1 official ROS 2 main docs target Iron.
- ROS 2 Humble is the standard LTS match for Ubuntu 22.04.
- ROS 2 Iron also supports Ubuntu 22.04, but it is not the long-term base and may introduce dependency differences.

## Recommended Strategy

Use Ubuntu 22.04 as the OS. Start with two workspaces:

```text
~/fcr_ws       ROS 2 Humble, original FCR + our adapter experiments
~/tron1_ws     TRON1 official stack, first test feature/humble; fall back to Iron if required
```

If TRON1 `feature/humble` builds and runs, prefer a Humble-only workflow. If it does not, keep TRON1 on Iron in a separate workspace or machine context and bridge only the small command interface.

## Docker Judgment

Docker is useful for repeatable builds, but it is not the first choice for early TRON1 hardware work because USB, serial, CAN, camera, Gazebo GUI, and robot networking add friction. Use native Ubuntu first. Add Docker later for CI-style compile smoke tests.
