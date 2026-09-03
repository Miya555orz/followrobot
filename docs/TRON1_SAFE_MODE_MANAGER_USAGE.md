# TRON1 安全模式管理使用文档

日期：2026-09-03

适用范围：TRON1 EDU / `WF_TRON1A`、FCR ROS 2 Humble、Gazebo 仿真、后续 Jetson/真机低速验收。

## 1. 新增功能

本次新增 FCR 侧安全模式管理节点：

```text
tron1_mode_manager_node
```

它不直接发布速度，只发布运动授权：

```text
/tron1/motion_authorized    std_msgs/Bool
```

TRON1 safety limiter 已增加第二道门控：

```text
enable_motion == true
并且 /tron1/motion_authorized == true
并且没有急停
并且输入没有 timeout
```

只有同时满足这些条件，`/fcr_tron/cmd_vel` 才可能输出非零。

## 2. 设计目的

官方遥控器开发者模式下，`L1 + 三角/Y` 会启动 `WheelfootController`。官方 `PointfootHardwareNode.cpp` 还会把摇杆输入按 `0.5` 比例发布到裸 `/cmd_vel`。

这条官方手柄路径可能很猛，不应该作为 FCR 的安全跟拍入口。

FCR 现在使用自己的安全状态机：

```text
状态机只授权
速度仍必须经过 limiter
真机默认 enable_motion=false
```

## 3. 安全架构

```text
模式请求 / 急停
        |
        v
tron1_mode_manager_node
        |
        v
/tron1/motion_authorized
        |
        v
tron1_safety_limiter_node
        |
        v
/fcr_tron/cmd_vel
        |
        v
TRON1 官方控制器
```

完整速度链路：

```text
Perception / Tracking / Follow
        |
        v
/fcr/cmd_vel_stamped
        |
        v
tron1_safety_limiter_node
        |
        v
/fcr_tron/cmd_vel
        |
        v
TRON1 official controller
```

## 4. 状态机

| 状态 | 含义 | 是否授权 TRON 运动 |
| --- | --- | --- |
| `IDLE` | 空闲 | 否 |
| `DEVELOPER_MODE` | 开发者模式入口 | 否 |
| `DEVELOPER_SELF_CHECK` | 开发者算法自检通过 | 否 |
| `REMOTE_STAND_READY` | 同款蹲起/站立准备 | 否 |
| `REMOTE_WALK_READY` | 同款行走准备 | 默认否 |
| `DEVICE_SELF_CHECK` | 设备自检通过 | 否 |
| `GIMBAL_FOLLOW` | 云台进入跟随 | 否 |
| `TRON_FOLLOW` | TRON1 底盘进入跟随 | 是，但仍受 limiter 限速 |
| `ESTOP` | 急停锁存 | 否 |

默认设计：只有 `TRON_FOLLOW` 授权底盘运动。

## 5. 模式请求

请求 topic：

```text
/tron1/mode_request    std_msgs/String
```

状态 topic：

```text
/tron1/mode_state      std_msgs/String
```

授权 topic：

```text
/tron1/motion_authorized    std_msgs/Bool
```

按顺序进入 TRON 跟随：

```bash
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: developer_mode}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: developer_self_check_pass}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: stand_ready}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: walk_ready}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: device_self_check_pass}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: gimbal_follow}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: tron_follow}"
```

退回：

```bash
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: back_to_gimbal_follow}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: back_to_walk}"
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: idle}"
```

急停：

```bash
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: estop}"
```

解除软件状态机急停：

```bash
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: clear_estop}"
```

复位到空闲：

```bash
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: reset}"
```

注意：`clear_estop` / `reset` 只处理 FCR 软件状态机。它不能替代 TRON1 物理急停、硬件 motor switch、遥控器急停锁存或官方错误码处理。

## 6. 启动仿真安全栈

只启动 FCR 侧安全状态机和 limiter：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash

ros2 launch bringup_pkg fcr_tron_safe_mode_sim.launch.py \
  enable_motion:=true \
  max_linear_x:=0.03 \
  max_angular_z:=0.10 \
  input_timeout_sec:=0.25
```

这个 launch 不连接真实 TRON1，不主动发布速度。

## 7. 启动官方 Gazebo + 自动验收

推荐直接运行：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh --with-gazebo
```

脚本会：

1. 启动 `fcr_tron_safe_mode_sim.launch.py`；
2. 启动官方 `robot_hw pointfoot_hw_sim.launch.py`；
3. 设置 `ROBOT_TYPE=WF_TRON1A`、`RL_TYPE=isaacgym`、`FCR_TRON_CMD_VEL_TOPIC=/fcr_tron/cmd_vel`；
4. 自动跑 32 组验收；
5. 结束后关闭测试进程。

如果本机 Gazebo 图形环境不可用，可以先跑不带 Gazebo 的 topic 安全仿真：

```bash
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh
```

## 8. 32 组验收覆盖内容

当前脚本实际跑 32 组，超过“至少 20 组”的要求：

1. 初始/复位后进入 `IDLE`
2. `IDLE` 不授权运动
3. `IDLE` 下输入大速度仍输出零
4. 非法跳转到 `TRON_FOLLOW` 被拒绝
5. 非法跳转后仍不授权
6. 进入开发者模式
7. 开发者算法自检通过
8. 进入同款蹲起/站立准备
9. 进入同款行走准备
10. 行走准备态默认仍不授权运动
11. 行走准备态大速度仍输出零
12. 设备自检通过
13. 云台进入跟随模式
14. 云台跟随态仍不授权 TRON 运动
15. 进入 TRON 跟随态
16. TRON 跟随态授权运动
17. `linear.x` 被限幅到 `0.03 m/s`
18. `linear.y` 被强制为 0
19. `angular.z` 被限幅到 `0.10 rad/s`
20. 输入 timeout 后输出自动归零
21. 外部急停强制进入 `ESTOP`
22. 外部急停后取消运动授权
23. 外部急停后输出归零
24. 急停锁存时拒绝继续进入 `TRON_FOLLOW`
25. 外部急停未解除时 `clear_estop` 无效
26. 外部急停未解除时仍不授权
27. `clear_estop` 后回到 `IDLE`
28. `clear_estop` 后仍不授权
29. 软件模式请求 `estop` 进入 `ESTOP`
30. 软件 `estop` 后不授权
31. `reset` 可回到 `IDLE`
32. `reset` 后输出保持零

## 9. 真机限制

真机不要直接使用仿真 launch。真机路径必须继续保持：

```text
enable_motion=false
```

只有完成以下检查后，才允许最低速短脉冲：

- TRON1 官方控制器确认订阅 `/fcr_tron/cmd_vel`；
- `/fcr_tron/cmd_vel` 只有 `tron1_safety_limiter` 一个发布者；
- 遥控器急停、硬件 motor switch、错误码状态已理解；
- 操作员一只手准备硬件停止；
- 首次速度不超过：
  - `linear.x = 0.01~0.02 m/s`
  - `angular.z = 0.03~0.05 rad/s`
  - 持续 `0.3~0.5 s`

## 10. 当前限制

- 本功能不能修改 TRON1 官方遥控器固件。
- 本功能不能阻止官方节点内部把真实遥控器输入发布到裸 `/cmd_vel`，所以真实遥控器开发者模式仍需谨慎。
- FCR 正式集成必须确保官方控制器消费 `/fcr_tron/cmd_vel`，不要走裸 `/cmd_vel`。
- Gazebo 已知存在零命令漂移和纯 yaw 横移问题；本验收只证明 FCR 安全状态机和 limiter 输出，不证明真实地面停止距离。
