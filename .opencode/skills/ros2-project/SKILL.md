---
name: ros2-project
description: Work safely inside this ROS2 workspace by checking package.xml, CMakeLists.txt, launch, config, nodes, topics, services, actions, and TF contracts.
compatibility: opencode
metadata:
  project: followrobot
---

## ROS 2 Checklist

Before editing:

- Inspect `git status --short --branch`.
- Identify the owning package under `src/<package_name>`.
- Read its `package.xml`, `CMakeLists.txt`, launch files, config files, and relevant node source.
- Check message/service/action dependencies before adding imports or includes.

After editing:

- If C++ or package metadata changed, run a focused `colcon build --packages-select <package>`.
- If launch/config changed, run a syntax or `--show-args` check when possible.
- If dependencies changed, mention whether `rosdep` or apt packages are required. Do not install system packages without human approval.
- Report topics, parameters, and frame IDs affected by the change.

## Commands

Use low-parallel builds to avoid exhausting the laptop or Jetson:

```bash
source /opt/ros/humble/setup.bash
cd /home/miya/follow_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 --packages-select <package>
```

For TRON1 official workspace checks:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
```

