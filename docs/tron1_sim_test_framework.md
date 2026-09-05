# TRON1 仿真测试框架

> 2026-09-05 安全注记：本文仅用于仿真和软件验收。Gazebo `WF_TRON1A + isaacgym` zero-command pose drift 仍是 blocker；仿真 PASS 不能替代真实 support-frame/架空测试。

本文用于组织后续大量 TRON1 迁移仿真测试。目标不是一次把自动跟拍跑起来，而是分层验收：每一层通过后，才允许进入下一层。

当前阶段建议采用“由小到大、由假到真、由话题到 Gazebo、由开环到闭环”的测试顺序：

```text
T0 环境预检
  ↓
T1 安全限速器话题级测试
  ↓
T2 TRON1 Gazebo 底盘桥接测试
  ↓
T3 运动模式与安全边界测试
  ↓
T4 FCR bringup 半链路测试
  ↓
T5 感知/云台/底盘联合测试
  ↓
T6 第一次实机前仿真验收
```

## 测试总原则

- 任何测试都不直接向 TRON1 裸 `/cmd_vel` 发自动跟拍速度。
- TRON1 控制器订阅 `/fcr_tron/cmd_vel`。
- FCR 输出先进入 `/fcr/cmd_vel_stamped`。
- `tron1_safety_limiter` 是所有底盘运动测试的唯一入口。
- 每次只改变一个主要变量，例如速度上限、角速度上限、超时时间、输入频率、控制源。
- 出现持续运动、倒地、Gazebo 发散、输出超过限幅、停止输入后不停车时，立即停止当前测试，不进入下一层。

## 测试前固定检查 T0

每次测试前先执行：

```bash
ps -ef | rg 'gazebo|gzserver|gzclient|robot_hw_node|pointfoot_node|rqt_robot_steering|ros2 topic pub|tron1_safety_limiter|component_container'
```

如果只看到 `rg` 自己，说明基本干净。若看到旧的 `ros2 topic pub` 或 `rqt_robot_steering`，先手动 `Ctrl+C` 关闭对应终端。

确认 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
source ~/limx_ws/install/setup.bash
echo $ROBOT_TYPE
echo $RL_TYPE
```

仿真阶段推荐：

```bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
```

## T1：安全限速器话题级测试

目的：不启动 Gazebo，只验证输入/输出类型、限幅、横移归零、斜坡、超时停车、急停。

启动限速器：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=true \
  max_linear_x:=0.05 \
  max_angular_z:=0.15 \
  input_timeout_sec:=0.30
```

检查类型：

```bash
ros2 topic info -v /fcr/cmd_vel_stamped
ros2 topic info -v /fcr_tron/cmd_vel
```

低速输入：

```bash
ros2 topic pub --rate 10 /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

观察输出：

```bash
ros2 topic echo /fcr_tron/cmd_vel
```

通过标准：

- `/fcr_tron/cmd_vel` 类型为 `geometry_msgs/msg/Twist`
- `linear.x` 能跟随输入，但不超过限幅
- `linear.y` 始终为 0
- 停止输入后 0.30s 左右输出回 0

## T2：TRON1 Gazebo 桥接测试

目的：验证 TRON1 Gazebo 能接收限速器输出，机器人只做低速、安全、短距离运动。

终端 A：

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
ros2 launch robot_hw pointfoot_hw_sim.launch.py fcr_cmd_vel_topic:=/fcr_tron/cmd_vel
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=true \
  max_linear_x:=0.05 \
  max_angular_z:=0.15
```

终端 C：

```bash
ros2 topic pub --rate 10 /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

通过标准：

- Gazebo 中机器人缓慢前进，没有突然冲刺。
- `/fcr_tron/cmd_vel` 输出稳定。
- 停止终端 C 后，机器人应很快停止。
- 终端 A 没有持续刷严重错误。

如果不想手敲 `ros2 topic pub`，可以使用中文控制台：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 run teleop_control_pkg tron1_chinese_teleop
```

启动后可以直接输入：

```text
直走
左转
右转
直走 左转
速度 0.04
左转 转速 0.12
停
急停
解除急停
帮助
退出
```

这个工具只发布 `/fcr/cmd_vel_stamped`，仍然必须经过 `tron1_safety_limiter` 后才会到 TRON1。

## T3：运动模式与安全边界测试

这一层专门测试“容易出事”的情况。

| 编号 | 测试内容 | 输入 | 通过标准 |
|------|----------|------|----------|
| T3-01 | 小前进 | `x=0.03, yaw=0` | 缓慢前进，停止输入后停车 |
| T3-02 | 小后退 | `x=-0.03, yaw=0` | 缓慢后退，停止输入后停车 |
| T3-03 | 原地小转 | `x=0, yaw=0.10` | 缓慢转向，不发散 |
| T3-04 | 弧线运动 | `x=0.03, yaw=0.08` | 能走弧线，姿态稳定 |
| T3-05 | 大输入限幅 | `x=1.0, y=1.0, yaw=2.0` | 输出不超过限幅，`y=0` |
| T3-06 | 超时停车 | 发布 2s 后停止 | 0.30s 左右输出回 0 |
| T3-07 | 急停 | 运动中发 `/safety/estop_state=true` | 立即输出 0 |
| T3-08 | 旧 publisher 残留 | 同时启动旧 `ros2 topic pub` | 能用 `ros2 topic info -v` 找到 publisher |
| T3-09 | 低频输入 | `--rate 1` | 因超时保护应呈现断续/停车，不持续跑 |
| T3-10 | 禁止运动门 | `enable_motion=false` | 有输入也始终输出 0 |

大输入限幅命令：

```bash
ros2 topic pub --rate 10 /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}}"
```

急停命令：

```bash
ros2 topic pub --once /safety/estop_state std_msgs/msg/Bool "{data: true}"
```

解除单独限速器急停：

```bash
ros2 topic pub --once /safety/estop_state std_msgs/msg/Bool "{data: false}"
```

## T4：FCR bringup 半链路测试

目的：启动 FCR 的 TRON 专用 bringup，但默认不打开自主跟拍。

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch bringup_pkg fcr_tron_full_follow.launch.py enable_motion:=false
```

通过标准：

- 旧三轮底盘驱动不启动。
- Sony 相机不启动。
- `/fcr/cmd_vel_stamped` 是 command_mux 的最终输出。
- `/fcr_tron/cmd_vel` 存在，但 `enable_motion=false` 时输出为 0。
- Orbbec 相关话题出现，例如 `/camera/depth/image_raw`。

确认命令：

```bash
ros2 topic list | sort | rg 'camera|cmd_vel|safety|gimbal|perception|auto|fcr'
ros2 topic info -v /fcr/cmd_vel_stamped
ros2 topic info -v /fcr_tron/cmd_vel
```

## T5：感知/云台/底盘联合测试

这一层还不能默认视为完成，因为当前没有 Sony 相机。建议拆成两个方向：

### T5-A：无 Sony 的安全桥测试

只验证：

- Orbbec 深度相机稳定 10Hz
- RS2 云台节点能启动或模拟启动
- TRON1 底盘只通过限速器接收手动/模拟速度

### T5-B：无 Sony 的自主目标来源改造

后续要实现完整跟拍，必须选择一个目标来源：

1. 使用 Orbbec 彩色流作为 YOLO 输入；
2. 使用外部检测节点发布 `/perception/tracks`；
3. 使用 mock target 做控制算法仿真，不测试真实视觉。

在没有完成这个选择前，不建议宣称“无 Sony 自动跟拍闭环已完成”。

## T6：第一次实机前仿真验收

进入真机前，至少完成以下仿真记录：

- [ ] T1 全部通过。
- [ ] T2 小前进、小后退、小转向通过。
- [ ] T3-05 大输入限幅通过。
- [ ] T3-06 超时停车通过。
- [ ] T3-07 急停通过。
- [ ] T4 `enable_motion=false` 安全启动通过。
- [ ] 记录 `max_linear_x=0.05`、`max_angular_z=0.15` 下的停止距离。
- [ ] 确认没有任何节点直接向 `/cmd_vel` 发送自动控制指令。

## 测试记录模板

每次测试建议记录到 `docs/tron1_sim_test_log.csv`：

```text
date,test_id,robot_type,rl_type,max_linear_x,max_angular_z,input,expected,result,notes
2026-08-31,T2-01,WF_TRON1A,isaacgym,0.05,0.15,"x=0.03 yaw=0","slow forward and timeout stop",PASS,"bridge output observed"
```

## 常用诊断命令

看话题类型和 publisher：

```bash
ros2 topic info -v /fcr/cmd_vel_stamped
ros2 topic info -v /fcr_tron/cmd_vel
ros2 topic info -v /cmd_vel
```

看输出频率：

```bash
ros2 topic hz /fcr_tron/cmd_vel
```

看当前输出：

```bash
ros2 topic echo /fcr_tron/cmd_vel
```

看是否还有残留进程：

```bash
ps -ef | rg 'gazebo|gzserver|gzclient|robot_hw_node|pointfoot_node|rqt_robot_steering|ros2 topic pub|tron1_safety_limiter|component_container'
```

## 失败处理规则

- 机器人持续运动：停止发布输入，若仍不停，发急停。
- `/fcr_tron/cmd_vel` 没有输出：先查限速器是否启动，再查输入话题类型是否是 `TwistStamped`。
- `/cmd_vel` 有多个 publisher：先停掉 `rqt_robot_steering` 和旧 `ros2 topic pub`，不要继续测试。
- Gazebo 卡住或控制器未启动：回到 T0，确认 `ROBOT_TYPE`、`RL_TYPE` 和 TRON 工作区环境。
- Orbbec 深度掉帧：确认 `lsusb -t` 中 Orbbec 是 `5000M`，并使用 `424x240@10Hz`。
