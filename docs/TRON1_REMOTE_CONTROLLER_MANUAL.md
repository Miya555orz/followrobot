# TRON1 遥控器操作手册

日期：2026-09-03

适用范围：TRON1 EDU / `WF_TRON1A`、LimX 官方 ROS 2 控制器、开发者模式、FCR 迁移项目。

本手册基于三类证据整理：

- 官方/本地配置：`/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/config/joystick.yaml`
- 官方/本地 ROS 入口：`/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/src/PointfootHardwareNode.cpp`
- 第一次开发者模式实机启动观测：[docs/TRON1_REMOTE_AND_SIM_SAFETY.md](TRON1_REMOTE_AND_SIM_SAFETY.md)

重要安全结论：

```text
L1 + X 是软件 stopController + 进程 abort。
它不是已经确认的电机阻尼 / 泄力 / 零力矩命令。

物理 motor switch / 硬件动作曾触发日志：
Motor in damping mode
```

## 1. 按键和摇杆映射

官方 `joystick.yaml` 映射如下：

| 实体按键 | SDK 编号 | 当前官方 ROS 节点中的含义 |
| --- | ---: | --- |
| A | button 0 | 已映射，但在 `PointfootHardwareNode.cpp` 中未发现特殊动作 |
| B | button 1 | 已映射，但未发现特殊动作 |
| X / 叉 | button 2 | 与 L1 同按：停止控制器并 abort 进程 |
| Y / 三角 | button 3 | 与 L1 同按：启动控制器 |
| L1 | button 4 | 启停组合键的修饰键 |
| R2 | button 5 | 已映射，但未发现特殊动作 |
| L2 | button 6 | 已映射，但未发现特殊动作 |
| R1 | button 7 | 已映射，但未发现特殊动作 |
| SELECT | button 8 | 已映射，但未发现特殊动作 |
| START | button 9 | 已映射，但未发现特殊动作 |
| 未知 / 未使用 | button 10 | SDK 可见槽位，`joystick.yaml` 未命名 |
| 未知 / 未使用 | button 11 | SDK 可见槽位，`joystick.yaml` 未命名 |
| Up / 上 | button 12 | 已映射，但未发现特殊动作 |
| Down / 下 | button 13 | 已映射，但未发现特殊动作 |
| Left / 左 | button 14 | 已映射，但未发现特殊动作 |
| Right / 右 | button 15 | 已映射，但未发现特殊动作 |
| MENU | button 16 | 已映射，但未发现特殊动作 |
| BACK | button 17 | 已映射，但未发现特殊动作 |

摇杆映射：

| 摇杆 | SDK 轴编号 | 官方 ROS 输出 |
| --- | ---: | --- |
| 左摇杆水平 | axes 0 | `/cmd_vel.linear.y = axes[0] * 0.5` |
| 左摇杆垂直 | axes 1 | `/cmd_vel.linear.x = axes[1] * 0.5` |
| 右摇杆水平 | axes 2 | `/cmd_vel.angular.z = axes[2] * 0.5` |
| 右摇杆垂直 | axes 3 | 配置中读取，但在 `PointfootHardwareNode.cpp` 中未发现输出动作 |

## 2. 开发者模式操作汇总

已确认的官方 ROS 组合：

| 操作 | 遥控器动作 | 代码证据 | 安全说明 |
| --- | --- | --- | --- |
| 启动控制器 | `L1 + Y/三角` | 调用 `startController(controller_name_)` | 可能让 TRON1 进入主动平衡/控制状态。必须撑稳机器人并清空周围空间。 |
| 停止控制器进程 | `L1 + X/叉` | 打 fatal 日志，调用 `stopController(controller_name_)`，然后 `abort()` | 只能当作软件停止/中止，不要当作泄力或阻尼。 |
| 手动前进/后退 | 左摇杆垂直 | 以 30 Hz 发布 `/cmd_vel.linear.x`，比例 `0.5` | 对早期 FCR 测试来说过猛，不建议作为安全限速路径。 |
| 手动横移 | 左摇杆水平 | 以 30 Hz 发布 `/cmd_vel.linear.y`，比例 `0.5` | 早期 TRON1 轮足测试应避免横移。 |
| 手动转向 | 右摇杆水平 | 以 30 Hz 发布 `/cmd_vel.angular.z`，比例 `0.5` | 只应在 safety limiter 路径证明可控后使用。 |
| 硬件阻尼 / 电机安全动作 | 物理 motor switch / 硬件动作 | 已观察到日志 `Motor in damping mode` | 目前最可信的急停/阻尼证据。运动前必须熟悉这个动作。 |

未知或当前未找到：

- 在当前 `PointfootHardwareNode.cpp` 中，未发现 `A`、`B`、`R1`、`R2`、`L2`、`START`、`SELECT`、方向键、`MENU`、`BACK` 的特殊动作。
- 本地 SDK 中未找到类似 `publishSensorJoy` / `publishJoystick` 的 API。
- 官方 ROS 节点里未找到已确认的遥控器真泄力 / 阻尼 / 零力矩按键。

## 3. 官网状态机图补充

用户提供的官网状态机截图显示，遥控器还有一层“整机状态机”语义。这里先作为官网图示记录，具体到本机固件和当前开发者模式仍需二次实机确认。

模式切换：

```text
开机后可进入：
  开发者模式：无预设遥控算法
  遥控模式：运行预设遥控算法

开发者模式 -> 遥控模式：R1 + Right
遥控模式 -> 开发者模式：R1 + Left
```

全局操作：

```text
L1 + □ / 方块 -> 空闲状态
按住左右摇杆 -> 急停
按遥杆右按键 -> 解除急停
R1 + 方向键上/下 -> 高度调节
```

开发者模式图示：

```text
空闲状态
  ├─ L1 + △ / 三角 -> 运行开发者算法
  └─ L1 + □ / 方块 -> 回到空闲状态
```

遥控模式图示：

```text
空闲状态
  └─ L1 + ○ / 圆圈 -> 蹲起状态
        └─ L1 + △ / 三角 -> 行走状态
              ├─ L1 + X / 叉 -> 蹲下状态
              └─ 摔倒后，L2 + △ / 三角 -> 从摔倒恢复到行走状态
```

注意：

- 上面是官网状态机图语义；本项目代码里已确认的是 `PointfootHardwareNode.cpp` 的 `L1 + Y` 启动控制器、`L1 + X` 停止控制器并 abort。
- 对 FCR 集成来说，不应依赖遥控模式的预设运动算法；FCR 应走 `/fcr/cmd_vel_stamped -> tron1_safety_limiter -> /fcr_tron/cmd_vel`。
- 急停/解除急停必须单独做实机低风险验证；不要把图示等同于已经在本机证明安全。
- 官网“按住左右摇杆 -> 急停”的全局语义尚未证明会在 FCR 连续发布 `/fcr_tron/cmd_vel` 时锁存并覆盖 FCR；当前结论是 `[UNVERIFIED]`，详见 `docs/TRON1_FCR_REMOTE_ESTOP_SEMANTICS_2026-09-05.md`。
- FCR 文档里说的“遥控/连续遥控”是电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄摇杆。手柄在 FCR 测试中只负责官方 controller 启停和物理急停/阻尼备份。

## 4. 只读遥控器监视

用下面命令学习真实遥控器输入。该工具不会发送电机命令：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A

ros2 run limxsdk_lowlevel pf_sensorjoy_monitor 10.192.1.2
```

期望输出示例：

```text
buttons[..., 4:1, ...]       # L1
buttons[..., 3:1, ...]       # Y / 三角
buttons[..., 2:1, ...]       # X / 叉
buttons[..., 4:1, 3:1, ...]  # L1 + Y/三角
```

这个工具只调用 `subscribeSensorJoy`，不调用 `publishRobotCmd`。

## 5. Gazebo 能不能模拟遥控器键位？

简短结论：

```text
官方 SDK / 本地 sim 暴露了 SensorJoy 订阅数据类型，
但当前没有找到官方提供的虚拟 SensorJoy 按键注入 API。
```

所以这里要区分两层：

### 现在可做：模拟遥控器效果

Gazebo 可以通过发布速度命令来测试控制器命令链路：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
export FCR_TRON_CMD_VEL_TOPIC=/fcr_tron/cmd_vel
export FCR_TRON_CMD_VEL_TIMEOUT_SEC=0.25

ros2 launch robot_hw pointfoot_hw_sim.launch.py \
  use_gazebo:=true \
  fcr_cmd_vel_topic:=/fcr_tron/cmd_vel \
  start_steering_gui:=false
```

然后再启动 FCR safety limiter，并向 limiter 输入端发布测试命令。这样可以验证命令接线、限幅、timeout 和停止行为。

### 当前不能直接做：虚拟按下 `L1 + Y` / `L1 + X`

官方节点通过下面方式接收真实遥控器事件：

```text
robot_->subscribeSensorJoy(&subscribeSensorJoyCallback)
```

本地 SDK / sim 头文件提供了 `subscribeSensorJoy`，但没有找到对应的 `publishSensorJoy`。因此，Gazebo 当前不能在 SDK 摇杆层面直接伪造“按下 L1 + Y”这种事件，除非额外写测试 shim 或修改官方节点。

## 6. 推荐的安全替代方案

不要先改真机路径。如果确实需要“键位级仿真”，建议只做仿真专用 shim：

```text
/sim_remote/buttons
  -> 仅测试用节点
  -> 调用等价的启动/停止逻辑，或发布等价 ROS 命令
  -> 仅 Gazebo 使用
```

但对当前 FCR/TRON1 迁移来说，更安全的优先级是：

1. 不依赖遥控器 joystick 直接发布的 `/cmd_vel` 作为 FCR 集成路径。
2. 让 TRON1 官方控制器只订阅 `/fcr_tron/cmd_vel`。
3. 所有 FCR 生成的底盘运动都必须经过 `tron1_safety_limiter`。
4. 用 Gazebo 验证 limiter 行为，不用 Gazebo 证明真实硬件阻尼。

## 7. 操作员速查卡

任何实机运动前：

```text
必须知道物理 motor switch / 硬件停止位置。
一只手随时准备按硬件停止。
先以 enable_motion=false 启动。
不要直接启动 Sony + RS2 + TRON 全链路跟拍。
```

已知动作：

```text
L1 + Y/三角       -> 启动 WheelfootController
L1 + X/叉         -> 软件 stopController + abort 进程
L1 + □/方块       -> 官网图示：回空闲状态，待本机确认
左右摇杆按下       -> 官网图示：急停；对 FCR 连续命令覆盖权仍为 [UNVERIFIED]
遥杆右按键         -> 官网图示：解除急停，待本机确认
R1 + Right/Left   -> 官网图示：开发者模式/遥控模式切换，待本机确认
左摇杆垂直        -> 前进/后退 cmd_vel
左摇杆水平        -> 横移 cmd_vel
右摇杆水平        -> 转向 cmd_vel
物理 motor switch -> 已观察到的阻尼路径
```
