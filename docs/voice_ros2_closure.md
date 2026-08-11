# FCR 语音控制 ROS 2 收口说明

## 1. 本阶段边界

本阶段只收口 Jetson 端 ROS 2 执行链。语音识别、自然语言解析和分类模型部署在独立计算机，后续单独实现。

独立计算机只能向 `/external/voice_command` 发布结构化候选意图，不能直接发布底盘、云台或相机执行指令。Jetson 根据当前状态、置信度、命令时效和安全约束决定是否执行。

```text
独立计算机
ASR → 意图分类 → /external/voice_command（候选）
                         ↓
Jetson voice_command_dispatcher_node（最终仲裁）
                         ↓
system_mode_manager / Sony服务 / 运镜Action / MANUAL_JOG
                         ↓
command_mux → 底盘与云台驱动
```

## 2. 唯一顶层状态

`system_mode_manager_node` 是顶层业务模式的唯一状态源，发布可靠且可保留最后值的 `/system/state`。

顶层模式：

- `STANDBY`：待机，可接受连续遥控和有界语音微调。
- `FOLLOW`：自动跟随，只允许跟随参数调整和相机录像控制，拒绝底盘/云台微调。
- `CINEMATIC`：运镜模式，子状态为 `READY` 或 `EXECUTING`。

紧急停车是覆盖状态。紧急停车生效时强制进入 `STANDBY`，且拒绝进入跟随或运镜。

模式切换服务：

```text
/system/set_mode    vision_servo_msgs/srv/SetSystemMode
/system/state       vision_servo_msgs/msg/SystemState
```

## 3. 语音指令准入矩阵

| 指令类型 | STANDBY | FOLLOW | CINEMATIC_READY | CINEMATIC_EXECUTING |
|---|---:|---:|---:|---:|
| 紧急停车/停止全部 | 允许 | 允许 | 允许 | 允许 |
| 底盘或云台微调 | 允许 | 拒绝 | 拒绝 | 拒绝 |
| 开始跟随 | 允许 | 幂等 | 拒绝 | 拒绝 |
| 停止跟随 | 拒绝 | 允许 | 拒绝 | 拒绝 |
| 调整跟随距离 | 拒绝 | 允许 | 拒绝 | 拒绝 |
| 进入运镜 | 允许 | 拒绝 | 幂等 | 拒绝 |
| 执行运镜动作 | 拒绝 | 拒绝 | 允许 | 拒绝/先取消当前动作 |
| 开始录像 | 允许 | 允许 | 允许 | 幂等 |
| 停止录像 | 允许 | 允许 | 允许 | 拒绝 |
| 拍照 | 允许 | 允许 | 允许 | 拒绝 |

所有接收/拒绝结果通过 `/voice/dispatch_status` 发布，拒绝原因是机器可读字符串。

## 4. Sony 相机控制闭环

Sony 节点复用现有 CRSDK 会话，不允许同时运行 RemoteCli 或 SimpleCli 抢占相机。

```text
/sony/set_recording     vision_servo_msgs/srv/SetCameraRecording
/sony/take_photo        std_srvs/srv/Trigger
/sony/recording_status  vision_servo_msgs/msg/CameraRecordingState
```

录像服务返回“命令是否被 SDK 接受”，最终状态以相机 `RecordingState` 属性为准。状态机包含：

```text
STOPPED → STARTING → RECORDING
RECORDING → STOPPING → STOPPED
任何命令失败或确认超时 → ERROR
```

运镜管理器自身具有录像互锁，因此语音、控制台或其他 Action 客户端都不能绕过：若尚未录像，Jetson 先请求开始录像，只有收到 `RECORDING` 确认后才执行运动；等待超时则安全终止动作。运镜执行期间，Sony 服务层也会拒绝停止录像。若录像由运镜管理器自动开启，则退出运镜模式时自动释放并停止录像。

## 5. 跟随距离接口

```text
/follow/set_distance    vision_servo_msgs/srv/SetFollowDistance
```

允许范围为 `0.5–8.0 m`。服务同时更新 `servo_manager` 的 `desired_depth` 参数和当前控制器目标，避免只改参数但当前控制目标不更新。

## 6. 启动策略

完整 bringup 始终启动 `system_mode_manager_node`。语音执行链仍由 `enable_voice_control:=true` 控制，但默认不启动：

- Jetson 本地 wake-up/ASR；
- Jetson 本地分类模型；
- Windows 文本 HTTP 桥接。

后续独立计算机完成部署后，只需提供一个可靠、唯一的 `/external/voice_command` 发布源。

`start_fcr.sh` 中的 `FCR_ENABLE_VOICE=true` 现在只表示启动 Jetson 端候选意图准入、动作桥接和命令路由；它不再检查或激活 Jetson 本地 Python 虚拟环境、分类模型、嵌入模型或 HTTP 文本服务。

集成键盘控制台的待机、跟随和运镜切换也统一请求 `/system/set_mode`，不再绕过业务状态机直接进入运镜。连续遥控和单次微调仍是 `STANDBY` 内部的操作子状态。

## 7. Jetson 编译与静态验收

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to vision_servo_msgs sony_camera_pkg \
    external_control_pkg servo_control_pkg teleop_control_pkg bringup_pkg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WAKE_UP_NODE=OFF

source ~/ros2_ws/install/setup.bash
```

启动实机后检查接口：

```bash
ros2 node list | grep -E 'system_mode_manager|voice_command_dispatcher|sony_camera'
ros2 topic echo /system/state --once
ros2 topic echo /sony/recording_status --once
ros2 service type /system/set_mode
ros2 service type /sony/set_recording
ros2 service type /sony/take_photo
ros2 service type /follow/set_distance
```

本阶段不验证外部分类模型准确率；该部分的输入协议、置信度标定和网络重试策略在独立计算机部署阶段处理。
