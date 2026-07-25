# 语音控制云台与底盘实机测试

本文固化当前已经跑通的实机流程：

```text
控制台自然语言
  -> 双层意图模型
  -> /external/voice_command
  -> voice_command_dispatcher_node
       |- /voice/gimbal_command
       |    -> voice_gimbal_nudge_node
       |    -> /voice/cmd_gimbal
       |    -> command_router_node
       |    -> /cmd_gimbal
       |    -> gimbal_driver
       |    -> DJI RS2
       |
       `- /voice/chassis_command
            -> voice_chassis_nudge_node
            -> /voice/cmd_vel
            -> chassis_command_router_node
            -> /cmd_vel
            -> chassis_driver
            -> LeKiwi
```

本流程不包含麦克风、唤醒词和 ASR。操作员在 Jetson 终端输入自然语言，
双层模型输出结构化意图，然后驱动真实云台和底盘。

## 1. 已验证的硬件与环境

- Jetson Orin Nano，Ubuntu 22.04，ROS 2 Humble。
- DJI RS2 通过 CANable `gs_usb` USB-CAN 接入。
- USB-CAN 的接口名可能是 `can0` 或 `can1`，禁止写死。
- LeKiwi 底盘通过 `/dev/ttyACM0` 接入。
- 三个 STS3215 舵机 ID 为 `7/8/9`，型号编号为 `777`。
- Python 运行环境为 `~/venvs/fcr_runtime`。
- 双层分类模型位于 `~/ros2_ws/models/classifier_v2`。
- BGE 模型位于 `~/ros2_ws/models/bge-base-zh-v1.5`。

## 2. 安全要求

第一次运行时必须：

1. 将三个底盘轮子架空。
2. 给云台留出完整转动空间并打开机械锁。
3. 确认底盘电池使用放电输出口，三个舵机均正常上电。
4. 准备随时关闭底盘和云台电源。
5. 不启动 `remote_platform.launch.py`、`command_mux_node` 或键盘遥控节点，
   避免多个节点同时发布最终控制话题。
6. 保持底盘低速参数，不在本次验收中提高速度。

软件急停不能替代物理断电急停。

## 3. 停止残留节点

```bash
pkill -INT -f keyboard_platform_teleop
pkill -INT -f command_mux_node
pkill -INT -f command_router_node
pkill -INT -f chassis_command_router_node
pkill -INT -f voice_command_dispatcher_node
pkill -INT -f voice_gimbal_nudge_node
pkill -INT -f voice_chassis_nudge_node
pkill -INT -f double_layer_console_voice_node
pkill -INT -f gimbal_driver_node
pkill -INT -f chassis_driver_node
sleep 2
```

确认没有遗留节点：

```bash
ros2 node list
```

## 4. 自动检测并配置云台 CAN

每次 Jetson 重启或 USB-CAN 拔插后都必须重新执行：

```bash
cd ~/ros2_ws/src/fcr_ros2_3
chmod +x tools/can/setup_gimbal_can.sh
./tools/can/setup_gimbal_can.sh
```

脚本根据内核驱动 `gs_usb` 自动识别 CANable，不会把 Jetson 板载
`mttcan` 误认为云台接口。记录脚本最后输出的接口，例如：

```text
GIMBAL_CAN_INTERFACE=can1
```

在后续每个需要启动云台的终端中设置：

```bash
export CAN_IF=can1
```

其中 `can1` 必须替换为脚本本次实际输出，不能沿用上一次启动的结果。

启动 ROS 前先确认 RS2 返回帧：

```bash
timeout 3s candump -tz "$CAN_IF",222:7FF
```

必须能看到 `0x222` 数据。如果没有输出，不要启动控制节点，应先检查
RS2 电源、USB-CAN、CAN-H/CAN-L 和接口状态。

## 5. 检查底盘

确认串口：

```bash
ls -l /dev/serial/by-id/
readlink -f \
  /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058979-if00
```

当前硬件应指向：

```text
/dev/ttyACM0
```

确认串口没有被其他进程占用：

```bash
sudo lsof /dev/ttyACM0
```

无输出表示可以继续。如果底盘舵机缺失，先解决供电或三芯总线连接，
不要修改舵机 ID，也不要运行 `setup_motors`。

## 6. 构建

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/venvs/fcr_runtime/bin/activate

export PYTHONNOUSERSITE=1
unset PYTHONHOME
export CUDA_VISIBLE_DEVICES=""

colcon build \
  --packages-select \
  vision_servo_msgs \
  robot_platform_pkg \
  external_control_pkg \
  voice_intent_pkg \
  --allow-overriding \
  vision_servo_msgs \
  robot_platform_pkg \
  external_control_pkg \
  voice_intent_pkg \
  --cmake-args -DBUILD_WAKE_UP_NODE=OFF

source ~/ros2_ws/install/setup.bash
```

确认模型入口已经安装：

```bash
ros2 pkg executables voice_intent_pkg |
grep double_layer_console_voice_node
```

## 7. 启动真实硬件驱动

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export CAN_IF=can1  # 替换为检测脚本本次实际输出

ros2 launch robot_platform_pkg platform.launch.py \
  use_sim:=false \
  enable_chassis:=true \
  enable_imu:=false \
  can_interface:="$CAN_IF" \
  gimbal_control_mode:=incremental_position
```

启动日志必须满足：

- 底盘 ID `7/8/9` 均响应；
- 云台连接到检测脚本给出的 `gs_usb` 接口；
- 没有底盘串口超时；
- 没有 CAN 初始化错误。

## 8. 硬件状态门禁

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 lifecycle get /gimbal_driver
ros2 topic echo /gimbal/status --once
```

必须满足：

```text
active [3]
connected: true
rx_count: 大于 0
last_rx_age_sec: 接近 0
```

如果出现：

```text
connected: false
last_rx_age_sec: -1.0
rx_count: 0
```

说明驱动没有收到 RS2 返回帧。重新执行 CAN 检测脚本，禁止继续发送控制命令。

检查底盘反馈：

```bash
ros2 topic echo /chassis/odom_raw --once
```

## 9. 启动语音执行层

终端 3：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_intent_classifier:=false \
  start_dispatcher:=true \
  start_command_router:=true \
  start_chassis_control:=true \
  start_keyboard_node:=false \
  right_yaw_sign:=1.0
```

应启动：

- `voice_command_dispatcher_node`
- `voice_gimbal_nudge_node`
- `command_router_node`
- `voice_chassis_nudge_node`
- `chassis_command_router_node`

如果云台左右方向相反，停止该 launch 后将
`right_yaw_sign:=1.0` 改成 `right_yaw_sign:=-1.0`。

## 10. 启动双层模型控制台

终端 4：

```bash
source /opt/ros/humble/setup.bash
source ~/venvs/fcr_runtime/bin/activate
source ~/ros2_ws/install/setup.bash

export PYTHONNOUSERSITE=1
unset PYTHONHOME
export CUDA_VISIBLE_DEVICES=""

ros2 run voice_intent_pkg double_layer_console_voice_node --ros-args \
  -p model_root:=$HOME/ros2_ws/models/classifier_v2 \
  -p embedding_model_dir:=$HOME/ros2_ws/models/bge-base-zh-v1.5 \
  -p device:=cpu
```

看到 `BERT>` 后输入自然语言并按回车。模型终端应同时显示分类结果、
置信度和最终发布的控制意图。

## 11. 观察数据流

诊断终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

按需分别运行：

```bash
ros2 topic echo /external/intent_result
ros2 topic echo /external/voice_command
ros2 topic echo /voice/dispatch_status
ros2 topic echo /voice/gimbal_command
ros2 topic echo /voice/chassis_command
ros2 topic echo /cmd_gimbal
ros2 topic echo /cmd_vel
```

`/voice/dispatch_status` 中的 `accepted=true` 和 `reason=routed` 只表示意图
通过分流，不代表硬件已经执行。硬件结果必须结合 `/gimbal/status`、
底盘反馈和实物动作判断。

## 12. 云台测试

每次输入一句，等待动作结束后再输入下一句：

```text
云台向右一点
云台向左一点
云台向上看一点
云台向下看一点
云台停止
云台回中
```

预期：

| 输入 | 意图 |
|---|---|
| 云台向右一点 | `gimbal_nudge_right` |
| 云台向左一点 | `gimbal_nudge_left` |
| 云台向上看一点 | `gimbal_nudge_up` |
| 云台向下看一点 | `gimbal_nudge_down` |
| 云台停止 | `gimbal_stop` |
| 云台回中 | `gimbal_home` |

验收要求：

1. 意图被分发到 `gimbal`，不能进入 `chassis`。
2. `/cmd_gimbal` 出现对应方向的短时非零指令。
3. 动作结束后输出停止指令。
4. `/gimbal/status.connected` 始终为 `true`，`rx_count` 持续增长。

## 13. 底盘测试

保持轮子架空。第一次只输入：

```text
底盘向前移动一点
```

确认轮子方向正确并在约 `0.4s` 后停止，再依次输入：

```text
底盘向后移动一点
底盘向左移动一点
底盘向右移动一点
底盘向左转一点
底盘向右转一点
底盘停止
```

预期：

| 输入 | 意图 |
|---|---|
| 底盘向前移动一点 | `chassis_move_forward` |
| 底盘向后移动一点 | `chassis_move_backward` |
| 底盘向左移动一点 | `chassis_move_left` |
| 底盘向右移动一点 | `chassis_move_right` |
| 底盘向左转一点 | `chassis_turn_left` |
| 底盘向右转一点 | `chassis_turn_right` |
| 底盘停止 | `chassis_stop` |

默认点动参数：

```text
线速度：0.05 m/s
角速度：0.20 rad/s
持续时间：0.4 s
```

验收要求：

1. 意图被分发到 `chassis`，不能进入 `gimbal`。
2. `/cmd_vel` 只在短动作期间出现对应非零分量。
3. 约 `0.4s` 后 `/cmd_vel` 自动归零。
4. 方向错误时立即停止，不继续测试其他方向。

## 14. 急停

底盘运动期间执行：

```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

`/cmd_vel` 必须立即归零。解除：

```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: false}"
```

解除后必须等待下一条新指令，不能恢复之前的运动。

## 15. 最终验收

```bash
ros2 topic info /external/voice_command -v
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_gimbal -v
```

必须满足：

- `/external/voice_command` 只有一个模型发布者；
- “云台向右”只进入云台链路；
- “底盘向右”只进入底盘链路；
- 底盘点动结束后自动停止；
- 云台始终保持 `connected: true`；
- CAN 接口来自检测脚本，不依赖固定的 `can0/can1`；
- 急停能够覆盖语音运动命令；
- 没有旧键盘节点或 `command_mux_node` 抢占最终话题。

## 16. 结束测试

先发送急停：

```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

然后在各 launch 和模型终端使用 `Ctrl+C`。确认节点退出后再关闭云台和底盘电源。

