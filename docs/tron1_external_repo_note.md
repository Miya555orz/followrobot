# TRON1 官方仓库本地改动说明

`followrobot` / `fcr_ros2_3` 仓库只包含 FCR 侧代码。当前迁移还依赖 TRON 官方工作区中的一个本地提交：

```text
仓库：~/limx_ws/src/tron1-rl-deploy-ros2
提交：8512578 Allow TRON1 cmd_vel topic override for FCR bridge
```

该提交的目的：让 TRON1 官方控制器不要固定订阅裸 `/cmd_vel`，而是可以通过 launch 参数订阅 FCR 安全限速器输出 `/fcr_tron/cmd_vel`。

2026-09-03 又发现一个实机前必须处理的安全点：如果 FCR 侧
`tron1_safety_limiter_node` 崩溃，TRON 官方 controller 原始逻辑会保留最后一次
`Twist` 速度命令。当前本地工作区已经在 `PointfootController`、
`SolefootController`、`WheelfootController` 中补了
`FCR_TRON_CMD_VEL_TIMEOUT_SEC` 超时清零逻辑，并通过编译。

最新复测结论：该 watchdog 会触发并清零速度意图，但 `WF_TRON1A + isaacgym`
Gazebo 在零命令下仍存在低速漂移；尝试过的 hard safe-stop（绕过 RL policy 后直接
hold/damping）会让模型姿态不稳定，已撤回。因此这条路径仍是实机前 blocker，不能把
controller-watchdog 当作实机 PASS。

FCR 侧 `tron1_safety_limiter_node` 已补 clean-shutdown zero burst：SIGINT/SIGTERM
shutdown 时发布 5 次零速度，topic 监听已确认尾包为 0。强制崩溃/kill -9 仍只能依赖
TRON controller watchdog 或硬件急停兜底。

另外，官方仿真 launch 已新增 `start_steering_gui` 参数并默认 `false`，避免 FCR
安全链路测试时自动启动 `rqt_robot_steering` 裸 `/cmd_vel` 输入旁路。需要 GUI 手动
控制时显式传入：

```bash
ros2 launch robot_hw pointfoot_hw_sim.launch.py start_steering_gui:=true
```

2026-09-03 还临时新增了一个只读遥控器监视工具：

```text
~/limx_ws/src/limxsdk-lowlevel/examples/pf_sensorjoy_monitor.cpp
```

它只调用 `subscribeSensorJoy` 打印 axes/buttons，不调用 `publishRobotCmd`，不发送电机命令。
用于确认实体遥控器按键映射：

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
ros2 run limxsdk_lowlevel pf_sensorjoy_monitor 10.192.1.2
```

实测确认：

```text
L1     = button 4
Y/三角 = button 3
X      = button 2
L1 + Y/三角 会启动 WheelfootController
L1 + X 是软件 stopController + abort，不是 damping/torque release
物理 motor switch / hardware action 会产生 Motor in damping mode
```

修改文件：

```text
robot_controllers/src/PointfootController.cpp
robot_controllers/src/SolefootController.cpp
robot_controllers/src/WheelfootController.cpp
robot_hw/launch/pointfoot_hw.launch.py
robot_hw/launch/pointfoot_hw_sim.launch.py
../limxsdk-lowlevel/examples/CMakeLists.txt
../limxsdk-lowlevel/examples/pf_sensorjoy_monitor.cpp
```

2026-09-03 本地未提交 watchdog 修改文件：

```text
robot_controllers/include/robot_controllers/PointfootController.h
robot_controllers/include/robot_controllers/SolefootController.h
robot_controllers/include/robot_controllers/WheelfootController.h
robot_controllers/src/PointfootController.cpp
robot_controllers/src/SolefootController.cpp
robot_controllers/src/WheelfootController.cpp
robot_hw/launch/pointfoot_hw_sim.launch.py
../limxsdk-lowlevel/examples/CMakeLists.txt
../limxsdk-lowlevel/examples/pf_sensorjoy_monitor.cpp
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
