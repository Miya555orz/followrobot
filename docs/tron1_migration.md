# TRON1 EDU 底盘迁移说明

本文记录将学长的 `fcr_ros2_3` 跟拍项目从三全向轮底盘迁移到逐际动力 TRON1 EDU 双轮足底盘的当前工程状态、启动链路和第一次实机测试前检查项。

> 安全原则：TRON1 EDU 体积和惯量明显大于原三全向轮底盘，不能让 FCR 跟拍算法直接发布到真机 `/cmd_vel`。必须经过 `tron1_safety_limiter`，并且第一次实机只允许低速、短距离、可急停测试。

## 当前进度

```text
TRON1 仿真环境      ████████░░ 80%
学长项目编译        ██████████ 100%
Orbbec 深度相机     █████████░ 90%
安全限速适配        ████████░░ 80%
TRON1 仿真联调      ██████░░░░ 60%
第一次实机准备      ████░░░░░░ 40%
```

已经确认：

- TRON1 官方工作区在 `~/limx_ws`，Ubuntu 22.04 + ROS 2 Humble。
- TRON1 仿真使用 `ROBOT_TYPE=WF_TRON1A`、`RL_TYPE=isaacgym`，机器人能在 Gazebo 中出现并由 `/cmd_vel` 控制移动。
- TRON1 官方控制器输入是 `geometry_msgs/msg/Twist`。
- FCR 工作区在 `~/follow_ws`，源码在 `~/follow_ws/src/fcr_ros2_3`，当前 14 个包已经能全量编译。
- FCR 旧底盘链路输出是 `geometry_msgs/msg/TwistStamped`，不适合直接接 TRON1。
- Orbbec Gemini 335 已在 USB 3.0 `5000M` 链路下跑通，推荐低负载模式 `424x240@10Hz`。
- 当前没有 Sony 相机，所以第一次实机准备阶段不依赖 Sony；自主视觉跟拍需要后续把检测输入改到 Orbbec 彩色流或其它目标源。
- 2026-08-31 已完成一次短时仿真冒烟测试：TRON1 Gazebo 启动到 `WheelfootController started`，`/fcr/cmd_vel_stamped` 输入经过限速器后，在 `/fcr_tron/cmd_vel` 看到 `geometry_msgs/msg/Twist` 输出。该测试验证了话题桥接和类型转换，不等价于完整视觉跟拍闭环验收。
- 2026-09-03 环境复查：FCR 侧 `robot_platform_pkg teleop_control_pkg bringup_pkg` 编译通过，TRON1 官方侧 `robot_controllers robot_hw` 编译通过；`pointfoot_hw_sim.launch.py` 已确认支持 `fcr_cmd_vel_topic` 参数。
- 2026-09-03 仿真安全复查：在已有 Gazebo 仿真中，`robot_hw_node` 订阅 `/fcr_tron/cmd_vel`；`tron1_safety_limiter` 在 `enable_motion=false` 时会强制输出全 0，在 `enable_motion=true` 时会将过大输入限幅，并在输入停止后自动回 0。
- 2026-09-03 一致性修正：当前 RS2 实测链路是外置 USB-CAN `can1`，TRON1/FCR 通信 launch 默认值已从旧的 `can0` 改为 `can1`。

## 工程结构速览

```text
fcr_ros2_3/
├── src/vision_servo_msgs/       # 自定义 msg/srv/action，含 GimbalCmd、PlatformState 等
├── src/perception_pkg/          # YOLO 检测、SORT/Kalman 跟踪、深度融合
├── src/servo_control_pkg/       # IBVS/PBVS/MVP 跟拍控制，产生底盘/云台候选控制量
├── src/teleop_control_pkg/      # command_mux、手动控制、急停和控制权切换
├── src/robot_platform_pkg/      # 旧三轮底盘、RS2 云台、IMU、里程计；新增 TRON1 安全限速器
├── src/bringup_pkg/             # 总启动文件
└── docs/                        # 项目文档
```

TRON1 相关新增/修改点：

- `src/robot_platform_pkg/src/tron1_safety_limiter_node.cpp`
- `src/robot_platform_pkg/config/tron1_safety_limiter.yaml`
- `src/robot_platform_pkg/launch/tron1_safety_limiter.launch.py`
- `src/bringup_pkg/launch/fcr_tron_full_follow.launch.py`
- TRON 官方 `robot_controllers` 支持用 `FCR_TRON_CMD_VEL_TOPIC` 环境变量改变控制器订阅的话题。
- TRON 官方 `robot_hw` launch 支持参数 `fcr_cmd_vel_topic:=/fcr_tron/cmd_vel`。

## 速度控制链路判断

原 FCR 链路：

```text
感知/跟踪
  ↓
servo_control_pkg / mvp_follow_controller_node
  ↓ geometry_msgs/msg/TwistStamped
/auto/cmd_vel
  ↓
teleop_control_pkg / command_mux_node
  ↓ geometry_msgs/msg/TwistStamped
/cmd_vel
  ↓
robot_platform_pkg / chassis_driver_node
  ↓ sendCommand(Twist) / emergencyStop()
旧三全向轮底盘
```

TRON1 迁移链路：

```text
FCR command_mux
  ↓ geometry_msgs/msg/TwistStamped
/fcr/cmd_vel_stamped
  ↓
robot_platform_pkg / tron1_safety_limiter_node
  ↓ geometry_msgs/msg/Twist
/fcr_tron/cmd_vel
  ↓
TRON1 官方 controller
  ↓
TRON1 仿真或真机
```

不能直接把 FCR `/cmd_vel` 接给 TRON1 的原因：

1. 类型不匹配：FCR 是 `TwistStamped`，TRON1 是 `Twist`。
2. 安全不够：旧 `chassis_driver_node` 的限速、超时、急停只保护旧三轮底盘，不会保护 TRON1 官方控制器。
3. 容易抢话题：`rqt_robot_steering`、`ros2 topic pub`、旧节点残留都可能继续向 `/cmd_vel` 发速度。

## TRON1 安全限速器

节点：`robot_platform_pkg/tron1_safety_limiter_node`

默认参数文件：`src/robot_platform_pkg/config/tron1_safety_limiter.yaml`

默认行为：

- 输入：`/fcr/cmd_vel_stamped`，类型 `geometry_msgs/msg/TwistStamped`
- 输出：`/fcr_tron/cmd_vel`，类型 `geometry_msgs/msg/Twist`
- `linear.x` 限制到 `±0.03 m/s`
- `angular.z` 限制到 `±0.10 rad/s`
- `linear.y` 默认强制为 `0`
- 50Hz 持续发布
- 0.25s 没有新输入则自动停车
- `enable_motion=false` 时永远输出 0，用于安全检查
- 订阅 `/safety/estop_state`，急停时立即输出 0

## 编译

FCR 工作区：

```bash
source /opt/ros/humble/setup.bash
cd ~/follow_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1
```

如果只改了 TRON1 适配相关内容，可以低成本编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/follow_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
  --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg
```

TRON 官方工作区：

```bash
source /opt/ros/humble/setup.bash
cd ~/limx_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
  --packages-select robot_controllers robot_hw
```

## 启动 TRON1 仿真 + 安全限速测试

启动前先检查有没有旧进程残留：

```bash
ps -ef | rg 'gazebo|gzserver|gzclient|robot_hw_node|pointfoot_node|rqt_robot_steering|ros2 topic pub|tron1_safety_limiter|component_container'
```

如果只看到 `rg` 自己，说明基本干净。不要习惯性 `rm -rf build install log`，也不要在不确定目标时乱杀进程。

终端 A：启动 TRON1 仿真，让 TRON 控制器订阅安全限速器输出。

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
ros2 launch robot_hw pointfoot_hw_sim.launch.py fcr_cmd_vel_topic:=/fcr_tron/cmd_vel
```

终端 B：单独启动限速器。第一次先保持 `enable_motion:=false`，确认话题没问题后再改成 `true`。

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py enable_motion:=false
```

确认输出话题类型：

```bash
ros2 topic info -v /fcr_tron/cmd_vel
```

打开运动后低速测试：

```bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=true max_linear_x:=0.05 max_angular_z:=0.15
```

发送一个很小的前进速度：

```bash
ros2 topic pub --rate 10 /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

发送一个故意过大的速度，验证会被限幅：

```bash
ros2 topic pub --rate 10 /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}}"
```

另开终端观察输出：

```bash
ros2 topic echo /fcr_tron/cmd_vel
```

预期：

- `linear.x` 不超过设置的 `max_linear_x`
- `linear.y` 始终为 `0`
- `angular.z` 不超过设置的 `max_angular_z`
- 输出会逐渐变化，不会突然跳变
- 停止输入后约 0.25s 自动回到 0

急停测试：

```bash
ros2 topic pub --once /safety/estop_state std_msgs/msg/Bool "{data: true}"
```

单独测试限速器时，如需解除这个急停状态：

```bash
ros2 topic pub --once /safety/estop_state std_msgs/msg/Bool "{data: false}"
```

## FCR + TRON1 联合启动入口

FCR 侧提供了 TRON1 专用 bringup：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch bringup_pkg fcr_tron_full_follow.launch.py enable_motion:=false
```

这个 launch 的默认策略很保守：

- 禁用旧三轮底盘驱动
- 禁用 Sony 相机
- 禁用 YOLO 检测和跟踪
- 禁用 MVP 自主跟拍控制
- 启动 Orbbec/Gemini 深度链路
- 启动 TRON1 安全限速器，但默认不允许非零速度输出

等 TRON 仿真、话题和限速器都验证后，再逐步打开：

```bash
ros2 launch bringup_pkg fcr_tron_full_follow.launch.py \
  enable_motion:=true \
  max_linear_x:=0.05 \
  max_angular_z:=0.15
```

## 当前硬件话题关系

第一次实机计划硬件：

- TRON1 EDU 双轮足底盘
- DJI RS2 云台
- Orbbec Gemini 335 深度相机
- Jetson Orin Nano
- 暂无 Sony 相机

当前可落地链路：

```text
Orbbec Gemini 335
  ├─ /camera/depth/image_raw
  ├─ /camera/depth/camera_info
  └─ /camera/depth/metadata

DJI RS2 云台
  └─ robot_platform_pkg/gimbal_driver_node
      ├─ 接收 /cmd_gimbal 或相关 GimbalCmd
      └─ 发布云台状态

TRON1 底盘
  └─ TRON 官方 controller
      └─ 订阅 /fcr_tron/cmd_vel
```

需要注意：没有 Sony 相机时，旧 FCR 的 RGB 检测链路不能完整工作。第一次实机建议只做“相机稳定性 + 云台通信 + TRON 低速安全桥”测试，不做自动跟人高速闭环。后续如果要无 Sony 完整跟拍，需要把检测输入迁移到 Orbbec 彩色流，或者提供其它目标检测/跟踪来源。

## Orbbec 已解决的坑

- `install_udev_rules.sh` 出现 `$'\r': 未找到命令` 是脚本 Windows 换行导致的，不是相机坏了。
- udev 规则已经安装到 `/etc/udev/rules.d/99-obsensor-libusb.rules`。
- 充电线/USB2 链路会显示 `480M`，深度图容易卡顿或降频。
- 正确的 USB3 链路在 `lsusb -t` 中应看到 Orbbec 设备接口为 `5000M`。
- Gemini 335 推荐当前低负载测试模式：`424x240@10Hz`。
- `320x240` 对当前设备/驱动不是有效深度模式。

## 第一次实机测试 checklist

测试前：

- [ ] 确认 TRON1 EDU 是 A 版还是 B 版；未确认前仿真继续用 `WF_TRON1A`。
- [ ] 确认 TRON 官方遥控器、电源、急停/失控处理方式都在手边。
- [ ] 清空机器人周围至少 2-3 米空间，远离玻璃、桌腿、线缆和人。
- [ ] 优先让轮子悬空或用支架做第一次空载速度验证。
- [ ] 确认没有旧的 `rqt_robot_steering`、`ros2 topic pub`、Gazebo、旧 FCR 底盘节点残留。
- [ ] 确认 TRON 控制器订阅的是 `/fcr_tron/cmd_vel`，不是裸 `/cmd_vel`。
- [ ] 确认 `tron1_safety_limiter` 已启动，`enable_motion` 初始为 `false`。
- [ ] 确认 `max_linear_x <= 0.03`、`max_angular_z <= 0.10` 再允许第一次非零输出。
- [ ] 确认停止输入后 0.25s 内 `/fcr_tron/cmd_vel` 会回 0。
- [ ] 确认 `/safety/estop_state` 发布 `true` 后输出立即为 0。

真机终端 A：TRON 官方真机控制。

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
ros2 launch robot_hw pointfoot_hw.launch.py fcr_cmd_vel_topic:=/fcr_tron/cmd_vel
```

真机终端 B：FCR 安全限速器，先锁死输出。

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py enable_motion:=false
```

确认安全后，才允许极低速：

```bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=true \
  max_linear_x:=0.05 \
  max_angular_z:=0.15 \
  input_timeout_sec:=0.25
```

如果任何现象不符合预期，立即停：

```bash
ros2 topic pub --once /safety/estop_state std_msgs/msg/Bool "{data: true}"
```

## 下一步工程任务

1. 在 Gazebo 中继续做可视化运动验收：FCR `/fcr/cmd_vel_stamped` → 限速器 → `/fcr_tron/cmd_vel` → TRON1 缓慢运动，并记录停止距离。
2. 给 RS2 云台做单独通信和低速姿态测试，确认不依赖 Sony。
3. 决定无 Sony 时的目标来源：Orbbec 彩色流、外部检测节点、手动/模拟目标，三选一。
4. 如果使用 Orbbec 彩色流，补充 YOLO 输入 remap 和相机内参/外参说明。
5. 第一次实机只测低速桥接和急停，不打开自主跟拍闭环。

大量仿真测试按 [TRON1 仿真测试框架](tron1_sim_test_framework.md) 执行，并把结果记录到 [tron1_sim_test_log.csv](tron1_sim_test_log.csv)。

注意：FCR 侧代码已能单独上传到 `followrobot`，但 TRON 官方控制器的话题覆盖能力属于另一个仓库，见 [TRON1 官方仓库本地改动说明](tron1_external_repo_note.md)。
