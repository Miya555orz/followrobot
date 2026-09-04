# TRON1 官方 controller / SDK 语义摸底

日期：2026-09-04

范围：只读分析本机 LimX 官方工作区：

```text
/home/miya/limx_ws/src/tron1-rl-deploy-ros2
```

本文件只解释官方 controller / SDK 的 stop、damping、zero-cmd 语义，不代表已经完成真机地面运动验收。

## 1. 一句话结论

```text
zero /cmd_vel 不是急停，不是泄力，也不是阻尼；
它只是给 RL policy 的“期望速度为 0”输入。
```

因此 FCR 不能依赖“发 0 速度”作为唯一安全停止手段。FCR 当前正确做法仍然是：

```text
状态授权 + limiter 限速 + timeout + estop 锁存 + 官方 watchdog + 物理停止
```

## 2. L1 + 三角/Y 的真实作用

证据：

```text
robot_hw/src/PointfootHardwareNode.cpp:83-89
```

官方 `SensorJoy` 回调中，`L1 + Y` 会调用：

```text
hw_loop_->startController(controller_name_)
```

对 `WF_TRON1A`，`controller_name_` 会被设置为：

```text
WheelfootController
```

证据：

```text
robot_hw/src/PointfootHardwareNode.cpp:139-148
```

这意味着：在开发者模式下，`L1 + 三角/Y` 不是“进入一个温和待机状态”，而是启动官方 Wheelfoot RL controller。该 controller 启动后会先进入 `STAND`，再进入 `WALK`。

## 3. L1 + X 的真实作用

证据：

```text
robot_hw/src/PointfootHardwareNode.cpp:92-98
```

官方代码中，`L1 + X` 会执行：

```text
hw_loop_->stopController(controller_name_)
abort()
```

所以它更接近：

```text
停止 controller + 终止 pointfoot_node 进程
```

它不是已证明的：

- damping；
- zero torque；
- 泄力；
- 硬件急停。

这也解释了之前实机体验：按下 `L1 + X` 没有产生你期待的“泄力/阻尼”现象，是符合代码语义的。

## 4. zero cmd_vel 的真实作用

证据：

```text
robot_controllers/src/WheelfootController.cpp:570-584
```

`cmdVelCallback()` 只是把 ROS `Twist` 写入：

```text
commands_(0) = linear.x
commands_(1) = linear.y
commands_(2) = angular.z
```

并把每个分量 clamp 到 `[-1.0, 1.0]`。

证据：

```text
robot_controllers/src/WheelfootController.cpp:488-495
robot_controllers/src/WheelfootController.cpp:430-433
```

这些 command 会被缩放后拼进 ONNX policy 输入。也就是说：

```text
cmd_vel = 0
  -> scaled_commands = 0
  -> policy 继续根据 IMU / 关节状态 / 历史动作 / 零期望速度输出动作
```

它不是“停止 controller”，也不是“停止所有 joint command”。

## 5. controller 启动后的状态

证据：

```text
robot_controllers/src/WheelfootController.cpp:68-81
robot_controllers/src/WheelfootController.cpp:83-99
robot_controllers/src/WheelfootController.cpp:183-215
```

`WheelfootController::onStart()` 会设置：

```text
mode_ = Mode::STAND
```

`onUpdate()` 中：

```text
STAND -> handleStandMode()
WALK  -> handleWalkMode()
```

`handleStandMode()` 的站立插值完成后会自动：

```text
mode_ = Mode::WALK
```

所以“启动 controller”之后，不是永久停在空闲或阻尼，而是最终进入 WALK policy 控制。

## 6. WALK 模式下的输出

证据：

```text
robot_controllers/src/WheelfootController.cpp:108-181
```

WALK 模式会持续：

1. 计算 observation；
2. 跑 encoder；
3. 跑 policy；
4. 得到 `actions_`；
5. 对非轮关节写 position / kp / kd / mode；
6. 对轮关节写 velocity / kd / mode。

这意味着即使 `cmd_vel = 0`，controller 仍然会为了平衡/姿态/策略输出关节和轮命令。

## 7. 官方 watchdog / timeout

证据：

```text
robot_controllers/src/WheelfootController.cpp:587-607
```

本地官方工作区已有 FCR 补丁：

```text
FCR_TRON_CMD_VEL_TIMEOUT_SEC
```

当 cmd_vel 超时后：

```text
commands_.setZero()
scaled_commands_.setZero()
```

但注意：这仍然只是把 policy 输入的期望速度置零，不等于 damping / zero torque。

因此官方 watchdog 是一层必要兜底，但不能替代 FCR limiter、软件 estop、硬件急停和物理阻尼。

## 8. damping / 泄力目前确认到什么程度

代码里存在控制参数：

```text
ControllerCfg.control.damping
ControllerCfg.control.wheel_joint_damping
```

它们是 controller 输出 joint command 时使用的 PD/轮关节阻尼参数，不是一个“遥控器泄力按钮 API”。

已观察到的真实 hardware damping 证据来自物理 motor switch / 硬件动作触发日志：

```text
Motor in damping mode
```

目前未在已读官方 ROS controller 代码里找到一个可由 FCR 直接调用的明确 `damping()` / `zero_torque()` / `lock()` ROS API。

## 9. 对 FCR 架构的要求

基于以上语义，FCR 侧必须遵守：

- `zero cmd_vel` 只能当“期望速度为零”，不能当急停。
- `L1 + X` 只能当“停止/中止官方进程路径”，不能当泄力。
- `TRON_FOLLOW` 授权必须是显式状态，不允许隐式从遥控器或视觉链路进入。
- `tron1_safety_limiter` 必须保持自己的急停锁存。
- `motion_authorized` 必须有新鲜度 timeout；mode manager 死亡后自动关门。
- 真机地面测试前，仍必须人工确认物理急停/阻尼可触达。

## 10. 当前 blocker

进入楼下草坪测试前还缺：

```text
[BLOCK] 官方或硬件层面的 damping / zero torque / lock API 尚未确认。
[BLOCK] zero cmd 只证明“零期望速度”，不证明“物理停止距离”。
[BLOCK] controller/SDK 进程死亡后的真机动力学后果还未在架空条件下验证。
```

下一步建议仍然是：

```text
架空 / 支架
enable_motion=false
确认 /fcr_tron/cmd_vel graph
确认物理停止动作
再做 0.01~0.02 m/s、0.3~0.5s 短脉冲
```
