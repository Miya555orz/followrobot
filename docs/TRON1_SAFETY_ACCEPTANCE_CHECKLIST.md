# TRON1 安全验收清单

目标：把 TRON1 工作从“能动”退回到“可控、可停、可证明安全”。

适用范围：TRON1 EDU / `WF_TRON1A`、PC 或 Jetson ROS 2 Humble、LimX 官方控制器、FCR safety limiter。

当前规则：在本清单通过前，不再进行 TRON1 实机运动。Gazebo 中零命令漂移是已知的官方仿真策略/物理 blocker，所以仿真只用于验证 topic 安全和控制链路，不用来证明真实停止距离。

## 安全架构

```text
PC / Jetson
  -> FCR 命令来源
  -> /fcr/cmd_vel_stamped              geometry_msgs/TwistStamped
  -> tron1_safety_limiter
  -> /fcr_tron/cmd_vel                 geometry_msgs/Twist
  -> TRON1 官方控制器
  -> TRON1
```

硬规则：

- TRON1 官方控制器必须订阅 `/fcr_tron/cmd_vel`。
- 不允许任何链路通过裸 `/cmd_vel` 直接驱动 TRON1。
- `tron1_safety_limiter` 必须是 `/fcr_tron/cmd_vel` 唯一允许的发布者。
- 真机默认必须是 `enable_motion=false`。

## 阶段 A：安全基线，不追求运动

运行：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./tools/tron1_bringup/tron1_safety_acceptance_check.sh
```

验收标准：

| ID | 检查项 | PASS 标准 |
| --- | --- | --- |
| A-01 | 测试前没有残留 `gazebo`、`pointfoot_node`、`tron1_safety_limiter` 或裸 `ros2 topic pub` 进程 | 没有非预期进程 |
| A-02 | `/fcr_tron/cmd_vel` 只有一个发布者 | 发布者是 `tron1_safety_limiter` |
| A-03 | TRON 控制器订阅 `/fcr_tron/cmd_vel` | 订阅者是 `robot_hw_node` / 官方控制器 |
| A-04 | TRON1 不使用裸 `/cmd_vel` 路径 | 官方控制器以 `fcr_cmd_vel_topic:=/fcr_tron/cmd_vel` 启动 |
| A-05 | `enable_motion=false` 强制输出 0 | 上游输入非零时，`/fcr_tron/cmd_vel` 仍为 0 |
| A-06 | `enable_motion=true` 时输出被限幅 | `linear.x <= 0.03`，`linear.y == 0`，`angular.z <= 0.10` |
| A-07 | 输入丢失 timeout | 输入停止后输出自动回到 0 |
| A-08 | `/safety/estop_state=true` | 输出立即回到 0 |
| A-09 | limiter 正常关闭 | SIGINT/SIGTERM 时发布零速度 burst |
| A-10 | limiter 或命令发布者崩溃 | 实机前必须理解官方控制器 watchdog / 硬件停止行为 |

备注：

- A-01 到 A-09 可以先在仿真/topic 级测试中验证。
- A-10 是进入真实低速运动前的关键 blocker。
- `tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh --with-gazebo` 是自动 topic/Gazebo/官方订阅验收；当前已通过 45/45。
- 官方 controller 语义见 [docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md](TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md)：zero cmd 只是 RL policy 的零期望速度，不是急停/泄力/阻尼。
- `tools/tron1_bringup/tron1_safety_acceptance_check.sh` 是真机前 read-only 闸门；没有 live ROS graph、没有确认物理急停/阻尼前，它应该输出 `BLOCK`，这不是脚本失败，而是在防止误把仿真 PASS 当成真机许可。
- `L1 + X` 是软件停止/中止，不是已证明的阻尼/泄力。
- 物理 motor switch / 硬件动作曾被观察到触发 `Motor in damping mode`。
- FCR 的“连续遥控”指电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄摇杆；手柄摇杆不作为 FCR 实机验收输入。
- 实机分步路线见 [docs/TRON1_REAL_TEST_STEP_CHECKLIST.md](TRON1_REAL_TEST_STEP_CHECKLIST.md)：先 PC 直连，再 Jetson 直连，最后 Jetson 加云台/相机。

## 阶段 B：官方控制器和遥控器状态机

当前只读结论见 [docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md](TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md)。已经确认：

- `L1 + 三角/Y` 启动 `WheelfootController`。
- `L1 + X` 是 `stopController()` + `abort()`，不是已证明的 damping / zero torque。
- zero `/cmd_vel` 只是 RL policy 的零期望速度输入，不是急停、泄力或阻尼。
- `WheelfootController` 启动后先进入 `STAND`，随后进入 `WALK` policy。

继续实机运动前，仍需回答这些硬件层问题：

- 官方是否提供 damping / lock / sit / zero-torque API？
- `/fcr_tron/cmd_vel` 发布者消失时会发生什么？
- SDK 进程死亡时会发生什么？

证据文件：

- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/src/PointfootHardwareNode.cpp`
- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_controllers/src/WheelfootController.cpp`
- [docs/TRON1_REMOTE_CONTROLLER_MANUAL.md](TRON1_REMOTE_CONTROLLER_MANUAL.md)
- [docs/TRON1_REMOTE_AND_SIM_SAFETY.md](TRON1_REMOTE_AND_SIM_SAFETY.md)
- [docs/tron1_external_repo_note.md](tron1_external_repo_note.md)

## 阶段 C：接口隔离

重新连接跟拍逻辑前，上层代码只能看到统一底盘命令接口。TRON1 细节必须留在 adapter / limiter / bringup 边界以下。

要求：

- follow / tracking 代码只发布一个通用底盘命令 topic。
- TRON1 adapter 将通用命令转换为 `/fcr/cmd_vel_stamped`，或直接转换为 limiter 输入。
- `tron1_safety_limiter` 仍然是硬门控。
- launch/config 选择 `omni` 或 `tron1`；perception 和 tracking 不包含 TRON1 专用逻辑。

设计文档：

- [docs/base_interface_tron1_adapter_design.md](base_interface_tron1_adapter_design.md)

## 阶段 D：真机最低风险运动

只有 A/B/C 通过后才允许：

1. 真机被可靠支撑，或轮子离地。
2. 确认硬件阻尼 / 急停动作可立即触达。
3. 先以 `enable_motion=false` 启动。
4. 确认上游输入非零时，`/fcr_tron/cmd_vel` 仍保持 0。
5. 只允许一次短脉冲：
   - `linear.x = 0.01~0.02 m/s`
   - 持续 `0.3~0.5 s`
6. 停止输入并验证停止。
7. 测试软件 estop。
8. 测试物理停止/阻尼。
9. 最后才测试极小 yaw：
   - `angular.z = 0.03~0.05 rad/s`

不要从 Sony + RS2 + TRON 全链路跟拍开始。

## 当前结论

```text
RS2 + Sony 视觉跟随已经成功。
TRON1 还不是跟拍底盘；它现在是处于 bringup 阶段的安全受控底盘接口。
```
