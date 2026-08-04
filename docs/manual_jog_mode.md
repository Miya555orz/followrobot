# MANUAL_JOG 手动微调模式

## 1. 控制边界

`MANUAL_JOG` 用于底盘或云台的小幅、有界、可取消直接运动。它与
`FOLLOW` 和 `CINEMATIC` 是互斥的顶层模式，不用于自动跟随中的构图偏置。

```text
FOLLOW/CINEMATIC/STANDBY
          |
          | 停止自动租约、速度归零
          v
   MANUAL_JOG/ENTERING
          v
   MANUAL_JOG/EXECUTING
          v
   MANUAL_JOG/SETTLING
          v
        STANDBY
```

- 进入时先向 `command_mux` 发送 `stop`，并尝试调用 `/cinematic/exit`。
- 执行时只使用 `/teleop/*` 手动通道，`command_mux` 仍是执行器唯一上游。
- 底盘位移和旋转使用 `/odom` 闭环判断完成度。
- 云台使用现有 `GimbalNudge` 相对角度命令，并使用 `/gimbal/status` 验证角度。
- 完成、取消、超时、反馈中断或急停都会先归零，再回到 `STANDBY`。
- 运镜管理器订阅 `/manual_jog/active`，微调期间拒绝新的运镜模式和任务。

## 2. Action 轴定义

`/manual_jog/execute` 使用 `vision_servo_msgs/action/ManualJog`：

| axis | 含义 | displacement 单位 |
|---:|---|---|
| 0 | 底盘前后 | m |
| 1 | 底盘左右 | m |
| 2 | 底盘偏航 | rad |
| 3 | 云台偏航 | rad |
| 4 | 云台俯仰 | rad |

正负方向遵循项目已标定的 `base_link` 和云台方向约定。

## 3. 构建与检查

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to vision_servo_msgs teleop_control_pkg \
    servo_control_pkg external_control_pkg bringup_pkg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source ~/ros2_ws/install/setup.bash
```

一键启动后，`remote_control.launch.py` 默认启动手动微调管理器：

```bash
ros2 run bringup_pkg start_fcr.sh --controller pbvs

ros2 action list -t | grep manual_jog
ros2 service list | grep manual_jog
ros2 topic echo /manual_jog/status --once --qos-durability transient_local
```

## 4. 安全小幅测试

底盘前进 10 cm：

```bash
ros2 action send_goal --feedback \
  /manual_jog/execute vision_servo_msgs/action/ManualJog \
  "{axis: 0, displacement: 0.10, max_speed: 0.06, timeout_sec: 6.0}"
```

底盘左移 10 cm：

```bash
ros2 action send_goal --feedback \
  /manual_jog/execute vision_servo_msgs/action/ManualJog \
  "{axis: 1, displacement: 0.10, max_speed: 0.06, timeout_sec: 6.0}"
```

云台向左 5 度（如实机方向相反，只将位移改为正值）：

```bash
ros2 action send_goal --feedback \
  /manual_jog/execute vision_servo_msgs/action/ManualJog \
  "{axis: 3, displacement: -0.0872665, max_speed: 0.20, timeout_sec: 5.0}"
```

云台抬高 5 度：

```bash
ros2 action send_goal --feedback \
  /manual_jog/execute vision_servo_msgs/action/ManualJog \
  "{axis: 4, displacement: 0.0872665, max_speed: 0.18, timeout_sec: 5.0}"
```

中途停止：

```bash
ros2 service call /manual_jog/stop std_srvs/srv/Trigger "{}"
```

任务结束后应同时满足：

```text
/manual_jog/status: mode=STANDBY, state=IDLE
/remote_control/status: mode=stop, active_source=stop
/cmd_vel: 全零
```

## 5. 语音路由

语音底盘/云台微调已改由 `voice_manual_jog_action_node` 发送 Action。旧的
`voice_chassis_nudge_node` 和 `voice_gimbal_nudge_node` 不再由默认语音启动文件启动，
因此不会再绕过顶层模式状态。

跟随或运镜内的“人物靠左一点”、“跟远一点”等指令不属于本 Action；
后续应通过构图偏置/期望距离参考值实现，不得转换为执行器速度。
