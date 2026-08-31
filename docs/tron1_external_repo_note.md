# TRON1 官方仓库本地改动说明

`followrobot` / `fcr_ros2_3` 仓库只包含 FCR 侧代码。当前迁移还依赖 TRON 官方工作区中的一个本地提交：

```text
仓库：~/limx_ws/src/tron1-rl-deploy-ros2
提交：8512578 Allow TRON1 cmd_vel topic override for FCR bridge
```

该提交的目的：让 TRON1 官方控制器不要固定订阅裸 `/cmd_vel`，而是可以通过 launch 参数订阅 FCR 安全限速器输出 `/fcr_tron/cmd_vel`。

修改文件：

```text
robot_controllers/src/PointfootController.cpp
robot_controllers/src/SolefootController.cpp
robot_controllers/src/WheelfootController.cpp
robot_hw/launch/pointfoot_hw.launch.py
robot_hw/launch/pointfoot_hw_sim.launch.py
```

使用方式：

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
ros2 launch robot_hw pointfoot_hw_sim.launch.py fcr_cmd_vel_topic:=/fcr_tron/cmd_vel
```

如果换了一台新电脑，只 clone `followrobot` 还不够；还需要在 `tron1-rl-deploy-ros2` 中应用同等改动，或者手动确认 TRON 控制器已经支持 `fcr_cmd_vel_topic` 参数。

检查方法：

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
ros2 launch robot_hw pointfoot_hw_sim.launch.py --show-args | rg fcr_cmd_vel_topic
```

如果能看到 `fcr_cmd_vel_topic`，说明 TRON 侧补丁存在。
