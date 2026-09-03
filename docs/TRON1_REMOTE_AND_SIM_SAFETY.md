# TRON1 遥控器与仿真优先安全说明

日期：2026-09-03

适用范围：TRON1 EDU / `WF_TRON1A`，本地 PC `/home/miya/follow_ws` + `/home/miya/limx_ws`。

本文记录第一次开发者模式实机 bringup 的观测结果，并定义下一步安全路线：优先使用仿真和遥控器监视，不继续让真机自由运动。

## 当前结论

第一次 TRON1 实机启动比预期更猛、更快。当前项目应暂停真实运动测试，退回到 Gazebo 仿真和遥控器熟悉阶段。

当前状态：

```text
[✓] PC <-> TRON1 Ethernet 链路可用。
[✓] TRON1 默认 IP 10.192.1.2 可通过 enp0s31f6 访问。
[✓] 官方 robot_hw 可以连接 TRON1。
[✓] LimX SDK SensorJoy 可以看到遥控器 axes/buttons。
[✓] L1 + triangle/Y 会启动 WheelfootController。
[✓] 物理 motor switch / 硬件动作可以让电机进入 damping mode。
[!] L1 + X 是软件 stopController + abort，不是电机阻尼 / 泄力。
[!] 真实运动体感对当前阶段过于激进。
[ ] 下一步继续 Gazebo / 仿真，不再直接扩大真机运动。
```

## 已观测遥控器映射

官方配置文件：

```text
/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/config/joystick.yaml
```

已观测/官方映射：

```text
A      = button 0
B      = button 1
X      = button 2
Y/△    = button 3
L1     = button 4
R2     = button 5
L2     = button 6
R1     = button 7
SELECT = button 8
START  = button 9
Up     = button 12
Down   = button 13
Left   = button 14
Right  = button 15
MENU   = button 16
```

摇杆轴：

```text
左摇杆水平 = axes[0]
左摇杆垂直 = axes[1]
右摇杆水平 = axes[2]
右摇杆垂直 = axes[3]
```

官方 ROS 行为：

```text
L1 + Y/△ = startController(WheelfootController)
L1 + X   = stopController(WheelfootController)，随后 abort()
```

重要：`L1 + X` 不是 damping / zero-torque 命令。它停止的是官方控制器进程路径。当前已观测到的 hardware damping 来自物理 motor switch / 硬件动作，不是 `L1 + X`。

更完整的遥控器操作手册见：[docs/TRON1_REMOTE_CONTROLLER_MANUAL.md](TRON1_REMOTE_CONTROLLER_MANUAL.md)。

## 只读遥控器监视

本地已添加 SDK 辅助工具：

```text
/home/miya/limx_ws/src/limxsdk-lowlevel/examples/pf_sensorjoy_monitor.cpp
```

它只订阅 `SensorJoy` 并打印 axes/buttons，不发布 `RobotCmd`，不会发送电机命令。

构建：

```bash
source /opt/ros/humble/setup.bash
cd /home/miya/limx_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 --packages-select limxsdk_lowlevel
```

运行：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
ros2 run limxsdk_lowlevel pf_sensorjoy_monitor 10.192.1.2
```

按键时的期望输出示例：

```text
buttons[..., 4:1, ...]       # L1
buttons[..., 3:1, ...]       # Y / 三角
buttons[..., 2:1, ...]       # X / 叉
buttons[..., 4:1, 3:1, ...]  # L1 + Y/三角
```

## 第一次实机启动日志证据

第一次启动过程中，日志显示控制器已经处于 active：

```text
Controller 'WheelfootController' is already active. Skipping start.
```

按下物理 switch / 硬件动作后，日志显示：

```text
Ethercat code: -1, msg: Motor in damping mode
Controller 'WheelfootController' stopped.
pointfoot_node exited with code -6
```

解释：物理开关/硬件动作确实让电机侧进入 damping mode。随后官方 ROS 节点把它视作 fatal 状态，停止控制器并退出。

## 仿真优先路线

在操作员熟悉遥控器启停行为、并重新测试仿真路径之前，不继续真实运动。

推荐下一步：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
export FCR_TRON_CMD_VEL_TOPIC=/fcr_tron/cmd_vel
export FCR_TRON_CMD_VEL_TIMEOUT_SEC=0.25

ros2 launch robot_hw pointfoot_hw_sim.launch.py \
  use_gazebo:=true \
  fcr_cmd_vel_topic:=/fcr_tron/cmd_vel \
  start_steering_gui:=false
```

然后单独测试 FCR limiter：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash

ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=false \
  max_linear_x:=0.01 \
  max_angular_z:=0.03 \
  input_timeout_sec:=0.25
```

只有理解仿真行为后，才继续恢复真实运动测试。

## 真实运动暂停规则

直到进一步确认前：

```text
不要以 enable_motion=true 运行真实 TRON1。
不要把 L1+X 当作阻尼或泄力。
不要启动 TRON1 全链路真人跟拍。
把物理 motor switch / 硬件停止作为首要 emergency action。
先用仿真学习控制器行为。
```
