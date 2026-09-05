# TRON1 + Jetson + DJI RS2 + Gemini depth full communication bring-up

日期：2026-09-01

目标：先建立完整通信链路，不让 TRON1 底盘运动。TRON1 机身重、惯量大，任何第一次实机测试都必须按“极低速、短时间、可急停、有人守护”的原则执行。

## 默认安全姿态

新入口：

```bash
ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py
```

默认值是通信优先、运动禁止：

- `start_tron_hw=false`：不自动连接 TRON1 官方硬件控制器。
- `enable_motion=false`：即使上游发速度，`tron1_safety_limiter` 也只输出 0。
- `max_linear_x=0.03`：第一次实机线速度上限 0.03 m/s。
- `max_angular_z=0.10`：第一次实机角速度上限 0.10 rad/s。
- `enable_lateral=false`：TRON1 首测禁止横移，`linear.y` 强制为 0。
- `enable_detection=false`、`enable_tracking=false`、`enable_mvp=false`：默认不启动视觉自主跟随。
- `gimbal_control_mode=incremental_position`：RS2 通信 bring-up 使用较保守的增量位置模式。

## 分层启动建议

### 1. 只启动 Jetson 侧通信链路

用于确认 Jetson 上 ROS2、RS2 CAN、深度相机、limiter、Foxglove 是否正常。此时不连接 TRON1 官方控制器。

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py \
  start_tron_hw:=false \
  enable_motion:=false \
  enable_gimbal:=true \
  rs2_can_interface:=can1 \
  start_depth_camera:=true \
  enable_depth_fusion:=true \
  use_foxglove:=true
```

验收点：

```bash
ros2 node list
ros2 topic info -v /fcr/cmd_vel_stamped
ros2 topic info -v /fcr_tron/cmd_vel
ros2 topic echo /gimbal/status --once
ros2 topic echo /camera/depth/image_raw --once
ros2 topic echo /camera/depth/camera_info --once
```

RS2 CAN 预检：

```bash
ip -details -statistics link show can1
```

要求：

- `/gimbal/status.connected` 为 true。
- `can_error_count` 不持续增加。
- 深度图和深度 CameraInfo 都有消息。
- `/fcr_tron/cmd_vel` 有且只有 `tron1_safety_limiter` 一个发布者。

### 2. TRON1 控制器接入检查，仍不做非零命令

只有当 TRON1 已经架空或周围留出防护空间时才打开 `start_tron_hw`。

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
source ~/limx_ws/install/setup.bash

ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py \
  start_tron_hw:=true \
  tron_robot_type:=WF_TRON1A \
  tron_rl_type:=isaacgym \
  enable_motion:=false \
  enable_gimbal:=true \
  rs2_can_interface:=can1 \
  start_depth_camera:=true \
  enable_depth_fusion:=true
```

验收点：

```bash
ros2 topic info -v /fcr_tron/cmd_vel
ros2 param get /tron1_safety_limiter enable_motion
```

要求：

- TRON 官方 `robot_hw_node` 订阅 `/fcr_tron/cmd_vel`，不是裸 `/cmd_vel`。
- `/tron1_safety_limiter enable_motion` 返回 `False`。
- 不按 `L1 + 三角/Y` 激活 controller，不推手柄摇杆，不发布任何非零上游速度命令。`enable_motion=false` 只约束 FCR limiter 输出，不约束官方 controller 自己的起立/进入 WALK 行为。

上游非零输入的安全 gate 验证只能放到架空/支架 Step 1：

```bash
ros2 topic pub --once /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}}"

ros2 topic echo /fcr_tron/cmd_vel --once
```

期望输出全 0。

### 3. 第一次非零底盘输出

这一步只在现场准备完成后做：

- 机器人架空或轮边无接触地面。
- 至少一人专门盯急停/电源。
- 周围没有人、线缆、桌腿、玻璃、相机三脚架。
- 先确认遥控器路径没有直接向裸 `/cmd_vel` 注入速度。

命令：

```bash
ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py \
  start_tron_hw:=true \
  enable_motion:=true \
  max_linear_x:=0.03 \
  max_angular_z:=0.10 \
  input_timeout_sec:=0.25
```

只允许发布极小短脉冲：

```bash
timeout 1s ros2 topic pub --rate 10 /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

停止后必须确认：

```bash
ros2 topic echo /fcr_tron/cmd_vel --once
```

期望输出全 0。

## 通信链路图

```text
Gemini depth camera
  -> /camera/depth/image_raw
  -> /camera/depth/camera_info
  -> depth_fusion_node

DJI RS2 over SocketCAN can1
  <-> gimbal_driver
  -> /gimbal/status
  <- /cmd_gimbal

FCR command_mux
  -> /fcr/cmd_vel_stamped
  -> tron1_safety_limiter
  -> /fcr_tron/cmd_vel
  -> TRON1 official robot_hw_node
```

禁止链路：

```text
任何节点 -> /cmd_vel -> TRON1
```

TRON1 只能订阅 `/fcr_tron/cmd_vel`。
