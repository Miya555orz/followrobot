# TRON1 实机分步验收清单

日期：2026-09-04

目标：把 TRON1 从“仿真安全链路通过”推进到“真实硬件低速可控”，但不直接进入 Sony + RS2 + TRON 全链路跟拍。

重要边界：任何脚本或清单都不能证明“100% 安全”。本清单只能把风险降到适合继续架空/支架、极低速、短脉冲分步验证的程度；现场操作员仍必须能立即触达物理停止/阻尼动作。

当前硬规则：

- FCR 链路里的“遥控”是电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄摇杆。
- TRON 手柄只保留两个用途：启动/停止官方控制器、物理急停/阻尼备份。
- FCR 测试时不使用手柄推杆控制底盘；不要让手柄摇杆绕过 limiter。
- TRON1 官方控制器必须订阅 `/fcr_tron/cmd_vel`。
- `/fcr_tron/cmd_vel` 唯一发布者必须是 `tron1_safety_limiter`。
- 裸 `/cmd_vel` 不能作为 FCR 到 TRON1 的实机入口。
- 每一步都先以 `enable_motion=false` 启动，确认输出为 0，再显式授权。
- A-10 必须逐项确认；旧的单个 `A10_CONFIRMED=yes` 不能单独作为真机运动许可。

## A-10 人工门

在任何真实底盘运动前，必须先完成并理解以下事实：

| 项 | 必须确认的事实 | 记录方式 |
| --- | --- | --- |
| A10.1 | 物理停止/急停动作位置已现场复核，操作员一只手可立即触达 | `A10_PHYSICAL_STOP_REHEARSED=yes` |
| A10.2 | 物理 motor switch / 硬件动作确实能进入 damping；日志或现象已复核 | `A10_DAMPING_OBSERVED=yes` |
| A10.3 | `L1 + X` 只是 `stopController()` + `abort()`，不是阻尼/泄力 | `A10_L1X_NOT_DAMPING_ACK=yes` |
| A10.4 | controller watchdog 只把期望速度清零，不等于物理停止/阻尼 | `A10_CONTROLLER_WATCHDOG_ACK=yes` |
| A10.5 | `WF_TRON1A + isaacgym` 零命令漂移仍是 blocker；Gazebo 不证明真实停止距离 | `A10_GAZEBO_ZERO_DRIFT_ACK=yes` |
| A10.6 | 已按本清单准备架空/支架、极低速短脉冲、停止方式和记录项 | `TRON1_REAL_TEST_CHECKLIST_ACK=yes` |

复查命令：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
source /home/miya/limx_ws/install/setup.bash

export A10_PHYSICAL_STOP_REHEARSED=yes
export A10_DAMPING_OBSERVED=yes
export A10_L1X_NOT_DAMPING_ACK=yes
export A10_CONTROLLER_WATCHDOG_ACK=yes
export A10_GAZEBO_ZERO_DRIFT_ACK=yes
export TRON1_REAL_TEST_CHECKLIST_ACK=yes
export A10_REVIEWED_BY="$(whoami)"
export A10_REVIEWED_AT="$(date -Is)"
./tools/tron1_bringup/tron1_safety_acceptance_check.sh
```

只有该 read-only gate 没有 `FAIL/BLOCK`，且现场仍满足硬件要求时，才进入第 1 步的 `enable_motion=false` 验证；仍不得直接做地面跟拍。若第 1 步已经启动 live bringup 并要复查 `/fcr_tron/cmd_vel` graph，可额外设置 `TRON1_LIVE_BRINGUP_INTENDED=yes`，表示当前 `tron1_safety_limiter_node`、`tron1_mode_manager_node` 和官方 controller 进程是本次现场分步检查的预期对象，而不是遗留进程。graph 中的官方型节点名只能证明拓扑，不替代“controller 确实连到 TRON1 硬件”的现场人工判断。

## 第 0 步：不动机器人准备项

这些项目应在第 1 步前完成；它们不是运动测试，不得发布速度命令：

1. 称量上装总重，记录 Jetson、RS2、Gemini、Sony、电池、支架和线缆总重。
2. 记录主要部件安装高度；能下移的重量先下移，避免把起立瞬态直接归因到 FCR 增益。
3. 在 Jetson 上做 TRON1 non-motion 网络检查：静态 IP、ping `10.192.1.2`、官方 node 只读连接证据。
4. 准备起立稳定等待记录：`L1 + 三角/Y` 激活官方 controller 后，先等待 `N` 秒并记录 IMU/现场观察；初始建议 `N=10s`，实际值以后续架空/支架观测为准。
5. 复核：在 `enable_motion=true` 前，操作员已经确认物理停止/阻尼可触达，且有人负责遥控器/物理停止。

当前不使用代码自动判断“已经稳定”：仓库内还没有 TRON1 IMU/姿态状态输入能证明起立瞬态结束。等待 `N` 秒只是规程门，不是安全保证。

## 三步路线

| 步骤 | 拓扑 | 实际验证 | 操作位置 |
| --- | --- | --- | --- |
| 1 | PC 直连 TRON1 | FCR 安全栈 + 官方 `robot_hw` 在 PC 上跑通；控制器订阅 `/fcr_tron/cmd_vel`；唯一发布者是 limiter；架空/支架下做第一次极低速短脉冲 | 键盘在 PC |
| 2 | Jetson 直连 TRON1 | 把第 1 步搬到 Jetson；PC 只通过 SSH 进入 Jetson 终端，不在 PC 本地跑 ROS 控制节点 | 键盘在 SSH 到 Jetson 的终端里 |
| 3 | Jetson 加云台/相机 | Jetson 同时跑 RS2、Gemini/Sony 和 TRON 通信；TRON 只做低速 yaw/距离修正，不做全速自动跟拍 | 键盘仍在 Jetson 进程所在终端 |

原先“四步计划”中的“PC -> Jetson -> TRON”和“Jetson 直连 TRON”本质上应合并：只要 FCR 控制台和 mux 跑在 Jetson，PC 就只是 SSH 观察端，不应依赖跨机器 DDS 来分发键盘。

## 第 1 步：PC 直连 TRON1

用途：在最简单网络拓扑下证明安全链和官方控制器接线正确。

硬件要求：

- TRON1 架空、上支架，或在 2 到 3 米空旷区域内，旁边有人扶稳。
- 物理 motor switch / 硬件停止可立即触达。
- 遥控器在手边，只用于官方 controller 启停和物理急停备份。
- 不启动 Sony、Orbbec、RS2、自动跟拍。

启动前只读检查：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
source /home/miya/limx_ws/install/setup.bash

./tools/tron1_bringup/tron1_safety_acceptance_check.sh
```

期望：

```text
FAIL=0
允许 BLOCK：未启动 live graph、A-10 逐项人工门未确认
```

启动 FCR 安全栈时，先保持：

```text
enable_motion=false
```

如果需要由手柄 `L1 + 三角/Y` 激活官方 controller，激活后先保持 FCR `enable_motion=false`，按第 0 步记录等待 `N` 秒；未完成等待和现场确认前，不允许把 FCR 运动门打开。

确认：

- 向 `/fcr/cmd_vel_stamped` 输入非零命令时，`/fcr_tron/cmd_vel` 仍为 0。
- `/tron1/limiter_state` 显示 `BLOCKED_ENABLE_MOTION_FALSE` 或其它明确阻塞原因。
- 只有确认官方控制器订阅 `/fcr_tron/cmd_vel` 后，才允许进入极低速短脉冲。

第一次短脉冲上限：

```text
linear.x <= 0.01~0.02 m/s
duration <= 0.3~0.5 s
angular.z 暂时不测
```

验收记录：

- 拓扑照片或文字描述。
- 启动命令。
- `/fcr_tron/cmd_vel` 发布者列表。
- `/fcr_tron/cmd_vel` 订阅者列表。
- `/tron1/limiter_state` 期望值。
- 手柄分工：只启停官方 controller，不推摇杆。
- 物理停止动作位置和实际效果。

## 第 2 步：Jetson 直连 TRON1

用途：确认最终上车计算平台能独立完成第 1 步。

原则：

- FCR 安全栈、`fcr_mode_console`、limiter、官方 controller 都跑在 Jetson。
- PC 不在本地跑 ROS 控制节点。
- PC 只负责 SSH、看日志、录屏、复制命令。
- 不依赖 PC 和 Jetson 共享 DDS graph 来分发键盘命令。

推荐连接方式：

```text
PC --SSH--> Jetson --Ethernet--> TRON1
```

键盘在哪里生效：

```text
fcr_mode_console 跑在哪个 ROS graph，键盘就在哪个进程生效。
第 2 步中，fcr_mode_console 应该跑在 Jetson 上。
你在 PC 终端里按键，本质上是通过 SSH 输入到 Jetson 进程。
```

不要做：

```text
PC 本地跑 fcr_mode_console
Jetson 跑 limiter / robot_hw
靠跨机器 DDS 共享话题来控制底盘
```

原因：当前 PC 网络曾出现 Mihomo/TUN 路由劫持，跨机器 ROS discovery 容易产生假象或延迟；实机低速验收应让控制台、mux、limiter 同机运行。

## 第 3 步：Jetson 加云台/相机

用途：验证 Jetson 能同时承载感知、RS2 云台和 TRON1 通信。

本步仍不是完整自动跟拍，只允许低速底盘修正：

```text
RS2 云台：负责目标居中、小角度快速修正
TRON1 底盘：只负责长期 yaw / 距离补偿
```

启动策略：

- 默认 `enable_motion=false`。
- 先验证 RS2 + Gemini/Sony + TRON 通信共存。
- 再验证 limiter 输出保持 0。
- 最后才允许短脉冲式低速底盘修正。

禁止：

- 一上来启动 Sony + RS2 + TRON 全链路自动跟拍。
- 让云台和底盘同时追同一个误差源。
- 丢目标后继续给 TRON1 非零速度。

## 键盘/手柄分工速查

| 工具 | 用途 | 是否用于 FCR 底盘遥控 |
| --- | --- | --- |
| `fcr_mode_console` | FCR 的键盘模式控制、连续遥控、deadman 停止 | 是 |
| TRON 手柄 `L1 + 三角/Y` | 启动官方 `WheelfootController` | 否，仅官方控制器启停 |
| TRON 手柄 `L1 + X` | 官方软件 `stopController()` + `abort()` | 否，不等同于泄力/阻尼 |
| 物理 motor switch / 硬件停止 | 已观察到 `Motor in damping mode` | 作为实机安全备份 |
| 手柄摇杆 | 官方 joystick 路径，可能发布裸 `/cmd_vel` | FCR 测试禁止使用 |

## 每步通过标准

每一步都必须记录：

```text
拓扑：
启动命令：
/fcr_tron/cmd_vel 发布者：
/fcr_tron/cmd_vel 订阅者：
/tron1/limiter_state：
enable_motion：
allow_tron_follow_motion：
上装重量/主要安装高度：
起立稳定等待 N 秒：
IMU/现场稳定观察：
物理急停/阻尼位置：
测试速度：
测试时长：
停止方式：
是否出现非预期运动：
```

只有上一项全部可解释，才进入下一项。这里慢一点，是为了后面能大胆一点。  
