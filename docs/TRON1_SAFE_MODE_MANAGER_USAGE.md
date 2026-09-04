# TRON1 安全模式管理使用文档

日期：2026-09-04

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

在 FCR 文档和 launch 里，“遥控/连续遥控”默认指电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄。键盘控制台跑在哪台机器，按键就在哪台机器的 ROS 进程里生效。真机第 2 步上 Jetson 时，PC 只应通过 SSH 输入到 Jetson 上的 `fcr_mode_console`，不要让 PC 本地和 Jetson 跨 DDS graph 混跑键盘。

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

limiter 状态 topic：

```text
/tron1/limiter_state        std_msgs/String
```

常见状态：

| 状态 | 含义 |
| --- | --- |
| `BLOCKED_ENABLE_MOTION_FALSE` | `enable_motion=false`，硬门控关闭 |
| `BLOCKED_MOTION_NOT_AUTHORIZED` | 状态机未授权 |
| `BLOCKED_AUTHORIZATION_TIMEOUT` | mode manager 死亡或授权信号过旧 |
| `BLOCKED_NO_ESTOP_SAMPLE` | 尚未收到急停源样本，按急停处理 |
| `BLOCKED_ESTOP_TIMEOUT` | 急停源样本过旧，按急停处理 |
| `BLOCKED_ESTOP_LATCHED` | limiter 急停锁存中 |
| `BLOCKED_INPUT_TIMEOUT` | 输入命令超时 |
| `INTENT_PASSING_ZERO_CMD` | 门控允许零速度命令；注意这不是泄力，实际输出仍以 `/fcr_tron/cmd_vel` 为准 |
| `INTENT_PASSING_LIMITED_CMD` | 门控允许限幅后的非零命令；实际输出仍以 `/fcr_tron/cmd_vel` 为准 |

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

让 limiter 看到“急停源当前为 false”的新鲜样本：

```bash
ros2 topic pub --rate 50 /safety/estop_state std_msgs/msg/Bool "{data: false}"
```

解除 limiter 自身急停锁存：

```bash
ros2 topic pub --once /tron1/limiter_clear_estop std_msgs/msg/Bool "{data: true}"
```

复位到空闲：

```bash
ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: reset}"
```

注意：`reset` 不能解除软件急停锁存；急停后必须显式 `clear_estop`。limiter 还要求 `/safety/estop_state` 持续有新鲜样本，否则 0.5 秒后会进入 `BLOCKED_ESTOP_TIMEOUT`。这些软件命令都不能替代 TRON1 物理急停、硬件 motor switch、遥控器急停锁存或官方错误码处理。

语义边界：

- `/safety/estop_state` 是 FCR `command_mux` 聚合出的软件急停状态，不是 TRON1 物理 motor switch，也不是手柄硬件急停。
- `/tron1/limiter_clear_estop` 是同一受控 ROS_DOMAIN 内的 limiter 软件恢复/维护入口，不是硬件安全边界；真机 limiter-only 部署不应把它作为唯一安全门，长期实机使用应放在隔离 domain/namespace 或加外层访问控制。

## 6. 启动仿真安全栈

只启动 FCR 侧安全状态机和 limiter：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash

ros2 launch bringup_pkg fcr_tron_safe_mode_sim.launch.py \
  enable_motion:=true \
  allow_tron_follow_motion:=true \
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

验收脚本默认使用独立仿真 domain：

```text
ROS_DOMAIN_ID=83
```

Python 验收脚本是 `ROS_DOMAIN_ID` 的单一真源：默认读取 `FCR_TRON_ACCEPTANCE_ROS_DOMAIN_ID`，未设置时使用 `83`；也可以显式传 Python 参数 `--ros-domain-id <1..232>`。脚本会在启动前打印最终生效值，并拒绝空值、`0`、前导零、非十进制或越界值。不要和真实 TRON1 bringup 共用同一个 ROS graph。

脚本会：

1. 启动 `fcr_tron_safe_mode_sim.launch.py`；
2. 启动官方 `robot_hw pointfoot_hw_sim.launch.py`；
3. 设置 `ROBOT_TYPE=WF_TRON1A`、`RL_TYPE=isaacgym`、`FCR_TRON_CMD_VEL_TOPIC=/fcr_tron/cmd_vel`；
4. 自动跑 47 组验收；
5. 结束后关闭测试进程。

如果本机 Gazebo 图形环境不可用，可以先跑不带 Gazebo 的 topic 安全仿真：

```bash
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh
```

真机前还要单独跑 read-only A 门检查：

```bash
./tools/tron1_bringup/tron1_safety_acceptance_check.sh
```

如果没有启动 live ROS graph、没有确认物理急停/阻尼，这个脚本输出 `BLOCK` 是正常且正确的；它的目的不是证明“可以动”，而是阻止把仿真验收误当成真机运动许可。

## 8. 47 组验收覆盖内容

当前脚本实际跑 47 组，超过“至少 20 组”的要求。

注意：真机进程/真机网络守卫、启动前 `/fcr_tron/cmd_vel` graph 预扫描、启动后 ROS graph 订阅者守卫都发生在主运动用例前；如果守卫发现真实 `pointfoot_node`、`robot_hw_node`、可达 TRON1 网络、启动前已有 `/fcr_tron/cmd_vel` endpoint，或非 Gazebo 模式下 `/fcr_tron/cmd_vel` 除 probe 外已有订阅者，脚本会在发布任何验收速度前直接拒绝运行。`--with-gazebo` 还会等待 graph 中出现 `/gazebo`，并且只把本次官方仿真的 `robot_hw_node` 作为允许的 TRON 订阅者。

新增两组硬门负向用例：

1. `enable_motion=false` 时，即使急停源新鲜、授权为 true、输入非零，limiter 输出仍为 0，状态为 `BLOCKED_ENABLE_MOTION_FALSE`。
2. `allow_tron_follow_motion=false` 时，即使状态机按顺序进入 `TRON_FOLLOW`，`/tron1/motion_authorized` 仍为 false。

主链路用例继续覆盖：

- 初始/复位后进入 `IDLE`，未授权时输出 0。
- 非法跳转到 `TRON_FOLLOW` 被拒绝。
- 开发者模式、同款蹲起/站立准备、同款行走准备、设备自检、云台跟随、TRON 跟随的顺序门。
- `linear.x`、`linear.y`、`angular.z` 的限幅和禁止横移。
- 输入 timeout 后输出自动归零。
- estop 样本超时后继续发命令仍输出 0。
- FCR 聚合软件急停 `/safety/estop_state`、软件 estop、limiter 急停锁存和显式 clear。
- mode manager 死亡后授权超时归零。
- `/fcr_tron/cmd_vel` 唯一发布者和本次官方 Gazebo `robot_hw_node` 订阅关系。
- Gazebo/robot_hw_sim 生命周期由验收脚本启动和回收；姿态漂移不作为真实地面运动证明。

Gazebo 练习脚本：

```bash
./tools/tron1_bringup/tron1_practice_gazebo.sh
./tools/tron1_bringup/tron1_practice_keyboard_gazebo.sh
```

这两个脚本只用于仿真练习；它们默认使用 `ROS_DOMAIN_ID=90`，拒绝空值、`0`、前导零、非十进制或越界 domain，启动前会检查 TRON1 默认地址不可达、无真实 `pointfoot_node`/`robot_hw_node` 进程、`/fcr_tron/cmd_vel` 和 `/tron1/mode_state` 无残留 endpoint，启动后会最多等待 60 秒直到看到 `/gazebo` 节点，才会推进状态机。推进后还会读回 `/tron1/mode_state`，确认进入 `TRON_FOLLOW` 后再打开控制台。退出时会按本脚本启动的进程组清理 Gazebo/robot_hw/FCR launch；只有脚本启动前 ROS daemon 没在运行时，cleanup 才会 stop daemon。若启动前残留 graph 被 BLOCK，先确认同 domain 没有其他 ROS 会话，再按脚本提示重启对应 `ROS_DOMAIN_ID` 的 daemon。

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

实机从 PC 到 Jetson 的分步路线见：[docs/TRON1_REAL_TEST_STEP_CHECKLIST.md](TRON1_REAL_TEST_STEP_CHECKLIST.md)。

## 10. 当前限制

- 本功能不能修改 TRON1 官方遥控器固件。
- 本功能不能阻止官方节点内部把真实遥控器输入发布到裸 `/cmd_vel`，所以真实遥控器开发者模式仍需谨慎。
- FCR 正式集成必须确保官方控制器消费 `/fcr_tron/cmd_vel`，不要走裸 `/cmd_vel`。
- Gazebo 已知存在零命令漂移和纯 yaw 横移问题；本验收只证明 FCR 安全状态机和 limiter 输出，不证明真实地面停止距离。
