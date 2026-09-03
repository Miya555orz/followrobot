# TRON1 Migration Gate Report

日期：2026-09-03

范围：本报告覆盖本机 Ubuntu 环境、`/home/miya/follow_ws`、`/home/miya/limx_ws`、Gazebo 仿真，以及 2026-09-03 晚间一次短暂 TRON1 真机通信/控制器激活观察。Jetson 未连接，完整真机运动未验收。

## 结论摘要

```text
Official Docs        PASS
SDK Check            PASS with caveat
Simulation           PASS
Basic Motion         PASS in Gazebo
TRON1 Adapter        PASS
Safety Tests         PARTIAL / BLOCKER
Low-speed Real Robot PARTIAL / PAUSED
Jetson Integration   NOT RUN
RS2 + Camera + TRON1 NOT RUN
Full Person Following NOT RUN
```

当前最重要阻塞：`tron1_safety_limiter` 正常运行时，限速、丢命令超时、急停都能输出 0；但故意杀掉 limiter 后，TRON 官方 controller 虽然已补 `cmd_vel` 超时清零日志，Gazebo 模型仍会继续低速位移。这个路径不能带到实机，必须继续做 controller/状态机/硬件急停级别的停止验证。

## Official Docs

状态：`PASS`

已读本地官方仓库：

- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2`
- `/home/miya/limx_ws/src/limxsdk-lowlevel`
- `/home/miya/limx_ws/src/tron1-gazebo-ros2`
- `/home/miya/limx_ws/src/tron1-robot-description`

关键证据：

- `robot_controllers/src/WheelfootController.cpp` 创建内部 ROS2 node `cmd_vel_node`。
- controller 订阅 `geometry_msgs/msg/Twist`。
- 默认 topic 是 `/cmd_vel`。
- 本地 FCR 补丁允许通过 `FCR_TRON_CMD_VEL_TOPIC` / `fcr_cmd_vel_topic` 改成 `/fcr_tron/cmd_vel`。
- `cmdVelCallback()` 使用：
  - `linear.x`
  - `linear.y`
  - `angular.z`

判断：

- 对 FCR 来说，真实高层入口是 `geometry_msgs/msg/Twist` 速度话题。
- SDK `RobotCmd` / joint command 是更低层接口，不应该让 FCR follow controller 直接触碰。
- joystick 是官方工具/人工输入路径，不是 FCR 自动跟拍迁移入口。

## SDK Check

状态：`PASS with caveat`

已确认：

- ROS2 当前为 Humble。
- `/home/miya/limx_ws` 下可见：
  - `limxsdk_lowlevel`
  - `limxsdk_sim`
  - `mrosbridger`
  - `pointfoot_gazebo`
  - `robot_controllers`
  - `robot_description`
  - `robot_hw`
  - `robot_visualization`
- `robot_hw` 提供 `pointfoot_node`。
- `limxsdk_lowlevel` 提供 `pf_joint_move`、`pf_groupJoints_move`，但这些是低层 joint 示例，不作为 FCR 入口。
- `WF_TRON1A + isaacgym` 的 `encoder.onnx` 和 `policy.onnx` 在 source/install 中都存在。
- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2` 当前 `main...origin/main [领先 1]`，本地补丁为：

```text
8512578 Allow TRON1 cmd_vel topic override for FCR bridge
```

注意：

- 非交互 shell 里 `ROBOT_TYPE` / `RL_TYPE` 不一定自动存在；运行 TRON1 仿真/实机 launch 前必须显式：

```bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
```

## Simulation

状态：`PASS`

验证命令：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
export FCR_TRON_CMD_VEL_TIMEOUT_SEC=0.25
ros2 launch robot_hw pointfoot_hw_sim.launch.py fcr_cmd_vel_topic:=/fcr_tron/cmd_vel
```

关键日志：

```text
Controller Name: WheelfootController
Set RL_TYPE to isaacgym
Successfully loaded ONNX models!
cmd_vel topic: /fcr_tron/cmd_vel, timeout: 0.250s
Controller 'WheelfootController' started.
```

判断：

- Gazebo 能启动。
- `WF_TRON1A + isaacgym` controller 能启动。
- controller 不是只露出话题，而是已经加载 ONNX 并进入 started 状态。

## Basic Motion

状态：`PASS in Gazebo`

测试内容：

- forward
- stop
- backward
- stop
- left
- stop
- right
- stop

方法：

- 通过 `tron1_safety_limiter` 输入 `/fcr/cmd_vel_stamped`。
- TRON controller 只订阅 `/fcr_tron/cmd_vel`。
- 用 `gz model -m pointfoot_entity -p` 读取 Gazebo 模型位姿。

结果：

- 每个方向命令后，Gazebo 模型位姿有可观察变化。
- stop / lost command 后，`/fcr_tron/cmd_vel` 回到 0。

注意：Gazebo 世界坐标变化方向受初始 yaw 影响，不等于直接看 x 增减判断 forward/backward。这里的 PASS 指仿真控制链路能驱动、能归零，不等于实机方向验收完成。

## TRON1 Adapter

状态：`PASS`

检查结果：

- `perception_pkg` 未混入 TRON1 专用逻辑。
- `servo_control_pkg` 仍输出通用速度命令。
- TRON1 细节集中在：
  - `robot_platform_pkg/tron1_safety_limiter_node`
  - `bringup_pkg` 的 TRON1 专用 launch
  - `teleop_control_pkg/scripts/tron1_chinese_teleop.py`
  - 文档
- 当前禁止裸 `/cmd_vel` 直连 TRON1，TRON 官方 controller 通过本地补丁订阅 `/fcr_tron/cmd_vel`。
- 官方仿真 launch 已新增 `start_steering_gui` 开关并默认关闭，避免 FCR 安全链路测试时自动启动 `rqt_robot_steering` 形成裸 `/cmd_vel` 人工输入旁路。

## Safety Tests

状态：`PARTIAL / BLOCKER`

已通过：

- `velocity clamp`：输入 `x=1.0, y=1.0, yaw=2.0`，输出被限制在极低速度附近，`linear.y=0`。
- `acceleration limit`：`tron1_safety_limiter_node` 使用 `max_accel_x/max_accel_y/max_accel_yaw` 对输出做斜坡限制。
- `lost command`：停止上游输入后，约 0.25s 后 `/fcr_tron/cmd_vel` 输出 0。
- `estop`：发布 `/safety/estop_state=true` 后，`/fcr_tron/cmd_vel` 输出 0。
- `clean shutdown zero`：`tron1_safety_limiter_node` 在 `SIGINT/SIGTERM` shutdown 时发布 5 次零速度 burst；topic 监听已确认尾包为 0。
- `single publisher`：测试时 `/fcr_tron/cmd_vel` 只有 `tron1_safety_limiter` 一个 publisher。

仍需注意：

- 强制崩溃/kill -9 无法依赖 limiter shutdown hook，必须由 TRON 官方 controller watchdog 或硬件急停兜底。
- TRON 官方 controller 原始行为会保持最后一次速度命令；本地已补 `FCR_TRON_CMD_VEL_TIMEOUT_SEC` 超时清零逻辑，并验证日志出现：

```text
cmd_vel timeout 0.250s exceeded; zeroing velocity command
```

后续尝试过两类 controller 级修复：

1. `hard safe-stop`：timeout 后绕过 RL policy，直接让腿部保持当前位置、轮子速度 0 + 阻尼。该方案会导致 `WF_TRON1A` Gazebo 姿态瞬间不稳定/接近翻倒，已撤回，不能用于真机。
2. `policy-zero`：timeout 后只清零 `commands_` / `scaled_commands_`，继续让官方 RL policy 负责平衡。该方案不会翻倒，但仍无法严格原地停止。

保留的当前本地代码是第 2 类保守方案：上游命令丢失时，官方 controller 自己把速度意图归零并继续运行 policy。

V2 复测结果：

```text
CRASH_V2_START              0.168832 -0.410741 ...
BEFORE_LIMITER_KILL         0.255072 -0.432594 ...
POINT7_AFTER_LIMITER_KILL   0.332837 -0.454545 ...
TWO7_AFTER_LIMITER_KILL     0.501078 -0.503699 ...
FOUR7_AFTER_LIMITER_KILL    0.619093 -0.541560 ...
AFTER_DIRECT_ZERO           0.722504 -0.578026 ...
```

V5 干净环境复测结果（`rqt_robot_steering` 默认未启动，`/fcr_tron/cmd_vel` 已无 publisher，controller watchdog 日志确认触发）：

```text
CRASH_V5_START              0.332683 -0.544855 0.766576 ...
BEFORE_LIMITER_KILL         0.395256 -0.566198 0.767095 ...
POINT7_AFTER_LIMITER_KILL   0.453186 -0.586042 0.766800 ...
TWO7_AFTER_LIMITER_KILL     0.552170 -0.623640 0.766881 ...
FOUR7_AFTER_LIMITER_KILL    0.635469 -0.657431 0.766993 ...
```

无输入基线也存在低速漂移：

```text
BASELINE_NOW                1.25050 -0.925810 0.767576 ...
BASELINE_PLUS2              1.29354 -0.955503 0.767617 ...
BASELINE_PLUS4              1.33777 -0.987331 0.767658 ...
```

因此，Gazebo 模型在零速度命令下仍有持续位移/漂移。原因可能是 RL/wheel policy、双轮足动态稳定、Gazebo 初始姿态/地面接触、或官方控制状态机缺少真正的硬停止模式。该路径必须继续调查，不能作为实机准入。

当前结论：

- FCR limiter topic 输出层安全测试 PASS，包括 clean shutdown zero burst。
- TRON 官方 controller 已能在输入消失后清零速度意图；但 Gazebo 物理位移仍不能作为真机安全停止 PASS。
- 真机前必须增加或验证 controller/hardware 级硬停止机制，使上游 publisher 丢失、limiter crash、节点异常退出时不会保持运动。

## Low-speed Real Robot

状态：`PARTIAL / PAUSED`

已观察：

- PC 到 TRON1 默认 IP `10.192.1.2` 的物理 Ethernet 链路已通，路由走 `enp0s31f6`，`ping` 成功。
- 官方 `pointfoot_node` 能连接 TRON1，加载 `WF_TRON1A` / `isaacgym` ONNX，并订阅 `/fcr_tron/cmd_vel`。
- FCR 安全链在 `enable_motion=false` 时，即使上游发布非零 `/fcr/cmd_vel_stamped`，`/fcr_tron/cmd_vel` 仍输出全 0。
- LimX SDK `SensorJoy` 可收到遥控器 axes/buttons；`L1 + Y/triangle` 会启动 `WheelfootController`。
- 按下物理 motor switch / hardware action 后，日志出现 `Motor in damping mode`，官方节点随后 `stopController` 并退出。

暂停原因：

- 第一次真机 controller 激活体感“太猛太快”，不适合作为继续实机运动的基础。
- `L1 + X` 只是软件 `stopController()` + `abort()`，不是 damping/torque release。
- `WF_TRON1A + isaacgym` Gazebo 仍存在零命令漂移，node crash/硬停路径仍需继续理解。

新增只读预检脚本：`tools/tron1_bringup/tron1_real_motion_path_preflight.sh`。该脚本检查路由、ping、ROS package、safe launch defaults 和已有 ROS graph topic；不发布任何速度命令。

遥控器和仿真优先路线见：`docs/TRON1_REMOTE_AND_SIM_SAFETY.md`。

## Jetson Integration

状态：`NOT RUN`

不执行原因：

- 用户明确先不接 Jetson。
- 上一轮 PC 侧发现以太网 `carrier=0` 且 `172.31.178.242` 路由被 Mihomo/TUN 捕获。

## RS2 + Camera + TRON1

状态：`NOT RUN`

准入条件：

- Jetson 能同时跑 Sony/Depth/RS2/TRON 通信。
- TRON 默认 `enable_motion=false`。
- `/fcr_tron/cmd_vel` 只有 limiter 一个 publisher。
- 云台控制和底盘控制需要明确职责分配，避免双控制器同时追同一误差导致振荡。

## Full Person Following

状态：`NOT RUN`

准入条件：

- 低速限定空间。
- 操作员值守。
- 物理急停可用。
- target lost 后云台和底盘都进入安全状态。
- 视觉算法不能绕过 safety limiter 直接驱动真机。
