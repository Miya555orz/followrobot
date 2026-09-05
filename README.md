# followrobot / fcr_ros2_3

![followrobot ROS2 system architecture](docs/figures/system_architecture.svg)

基于 ROS 2 Humble 的智能跟拍机器人项目：目标是在 Jetson Orin Nano 上运行 Sony/Orbbec 感知、DJI RS2 云台控制，并逐步接入 LimX TRON1 EDU 双轮足底盘。

一句话历史：本仓库源自学长的 FCR 视觉伺服项目，原底盘是 LEKIWI/三全向轮；当前开发重点已经切换到 `Miya555orz/followrobot` 的 Jetson + RS2 + Orbbec + TRON1 迁移。

当前活跃仓库：

```text
https://github.com/Miya555orz/followrobot
```

接手入口：

- [docs/HANDOFF.md](docs/HANDOFF.md)：完整工程交接
- [docs/CODEX_HANDOFF_PROMPT.md](docs/CODEX_HANDOFF_PROMPT.md)：给下一位 Codex 的启动 Prompt
- [docs/TRON1_REMOTE_AND_SIM_SAFETY.md](docs/TRON1_REMOTE_AND_SIM_SAFETY.md)：TRON1 遥控器、急停/阻尼和仿真优先说明
- [docs/TRON1_REMOTE_CONTROLLER_MANUAL.md](docs/TRON1_REMOTE_CONTROLLER_MANUAL.md)：TRON1 开发者模式遥控器操作手册
- [docs/TRON1_SAFETY_ACCEPTANCE_CHECKLIST.md](docs/TRON1_SAFETY_ACCEPTANCE_CHECKLIST.md)：TRON1 安全验收清单
- [docs/base_interface_tron1_adapter_design.md](docs/base_interface_tron1_adapter_design.md)：`base_interface + tron1_adapter` 设计
- [docs/TRON1_SAFE_MODE_MANAGER_USAGE.md](docs/TRON1_SAFE_MODE_MANAGER_USAGE.md)：TRON1 安全模式管理使用文档
- [docs/TRON1_SAFE_MODE_ACCEPTANCE_2026-09-04.md](docs/TRON1_SAFE_MODE_ACCEPTANCE_2026-09-04.md)：47 组 Gazebo/robot_hw_sim 安全验收记录
- [docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md](docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md)：TRON1 官方 controller/SDK stop、damping、zero-cmd 语义摸底
- [docs/TRON1_REAL_TEST_STEP_CHECKLIST.md](docs/TRON1_REAL_TEST_STEP_CHECKLIST.md)：TRON1 从 PC 直连到 Jetson 云台协同的实机分步验收清单
- [docs/TRON1_NEXT_STAGE_PLAN_2026-09-05.md](docs/TRON1_NEXT_STAGE_PLAN_2026-09-05.md)：第十五轮审查后下一阶段计划
- [docs/ai/OPENCODE_USAGE.md](docs/ai/OPENCODE_USAGE.md)：OpenCode 命令行与使用指南

## 当前最新状态

TRON1 真机已短暂进入开发者模式并激活过 controller，但体感过猛；当前策略改为：真机运动暂停，先回 Gazebo 仿真、遥控器熟悉和安全链路复核。

已确认的关键事实：

- Jetson Orin Nano CLB 已刷好 JetPack 6.2.3 / Jetson Linux R36.5.2，并从 NVMe 启动。
- DJI RS2 当前实测链路是 external USB-CAN `can1` / `gs_usb`，`/gimbal/status connected=true`。
- Orbbec Gemini 335 深度流已在 Jetson USB3 下跑通，约 424x240@10Hz。
- Sony UVC 图像链路、YOLO 检测、tracking、aim target、RS2 闭环跟拍已在 Jetson 实测跑通。
- Sony CRSDK 不在仓库，CRSDK 版 `sony_camera_node` 尚未启用。
- PC 到 TRON1 `10.192.1.2` 的 Ethernet / SDK 通信曾于 2026-09-03 打通；实机前必须重新跑只读 preflight。2026-09-05 本机最新检查发现路由被 `Mihomo`/policy table 捕获且 ping 不通，不能作为实机前提。
- TRON1 官方 `pointfoot_node` 已连接真机并加载 `WF_TRON1A` / `isaacgym` ONNX。
- LimX SDK `SensorJoy` 已读到遥控器 axes/buttons。
- `L1 + 三角/Y` 会启动 `WheelfootController`。
- `L1 + X` 是软件 `stopController()` + `abort()`，不是泄力/阻尼。
- FCR 侧已新增 `tron1_mode_manager_node`，`/fcr_tron/cmd_vel` 现在必须同时满足 `enable_motion=true`、`/tron1/motion_authorized=true` 且授权信号新鲜才可能非零。
- TRON1 安全模式管理已通过 47/47 组 Gazebo/robot_hw_sim 验收；已覆盖 enable_motion=false、allow_tron_follow_motion=false、mode manager 死亡后授权超时归零、limiter 急停锁存、estop 样本超时 fail-closed、limiter_state 诊断、官方控制器订阅关系。
- 官方 WheelfootController 语义已只读摸底：zero cmd 只是 RL policy 的零期望速度，不是急停/泄力/阻尼。
- 真机前 read-only A 门已拆成逐项人工确认：物理停止可触达、damping 证据、`L1+X` 语义、controller watchdog 后果、Gazebo 零漂 blocker 和实机分步 checklist 都必须显式确认。
- 真机前 read-only A 门脚本会在没有 live graph/逐项 A-10 确认时输出 `BLOCK`；若正在做 live bringup graph 复查，必须设置 `TRON1_LIVE_BRINGUP_INTENDED=yes`，但 Gazebo/robot_hw_sim/steering GUI/裸 topic pub 仍会被拦截。
- TRON1 只读实机运动路径预检现在会把代理/TUN/container/policy-table 路由或 `TRON_IP` ping 不通判为 `BLOCK` 并返回非零；这只是阻止继续实机准备，不会发布任何速度命令。
- `/fcr_tron/cmd_vel` 的 graph 检查只能证明唯一 limiter 发布者和官方型节点名订阅关系，不能替代“官方 controller 确实连接 TRON1 硬件”的现场人工确认。
- 第十五轮建议已转成下一阶段计划：先做上装重量/重心记录、Jetson 到 TRON1 non-motion 网络检查、起立稳定等待 `N` 秒的规程记录；non-motion 网络检查只到链路/IP/ping，不激活官方 controller；目标速度前馈、限速分档、depth fusion 外推先走设计/仿真审查，不直接改真机输出。
- 物理 motor switch / hardware action 会触发 `Motor in damping mode`。
- FCR 链路中的“遥控”指电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄摇杆；手柄只保留官方控制器启停、物理急停/阻尼备份。
- TRON1 不允许裸接旧 `/cmd_vel`；安全链路必须是 `/fcr/cmd_vel_stamped -> tron1_safety_limiter -> /fcr_tron/cmd_vel`。
- `WF_TRON1A + isaacgym` 官方 Gazebo pose 仍有零命令漂移/纯 yaw 横移；当前处理方式是把它保留为官方 sim-policy/physics blocker 并要求人工确认，不把 Gazebo pose 当作真机运动 PASS 条件。

## 模块级进度

```text
基础工程
  GitHub fork / 推送链路        ██████████ 100%  followrobot/main 已确认
  当前 README / handoff         ██████████ 100%  只描述当前状态，旧版本已压缩
  OpenCode 使用指南             █████████░  90%  命令行/安全边界已写，Harness 写入流待实战
  本地 ROS2 focused build       ██████████ 100%  robot_platform / teleop / bringup 通过

Jetson
  Jetson 刷机 / NVMe 启动       ██████████ 100%  JetPack 6.2.3
  Jetson ROS2 Humble            ██████████ 100%  基础环境可用
  PC <-> Jetson SSH             █████████░  90%  以太网 SSH 可用，注意 Mihomo/USB gadget 坑
  Jetson <-> TRON Ethernet      ███░░░░░░░  30%  PC 侧已通，Jetson 侧未实测

DJI RS2 云台
  USB-CAN can1 / gs_usb         ██████████ 100%  can1 可用
  RS2 ROS2 driver               ██████████ 100%  connected=true
  RS2 小角度手动控制            ██████████ 100%  yaw 微动验证完成
  RS2 视觉跟拍                  █████████░  90%  Sony UVC + 人像 + RS2 闭环已实测

相机 / 感知
  Orbbec Gemini 335 深度流      ██████████ 100%  低负载深度流已验证
  RS2 + Orbbec 共存             ██████████ 100%  已验证
  Sony UVC 图像链路             █████████░  90%  /dev/video8 -> /sony/image_raw
  YOLO 检测                     ████████░░  80%  person smoke test 已过
  tracking / aim target         ████████░░  80%  /perception/aim_target_2d 已验证
  Sony CRSDK 正式链路           ██░░░░░░░░  20%  SDK 未入库，节点未启用

TRON1 SDK / 控制器
  官方文档/仓库/launch 理解      █████████░  90%  入口、env、topic、ONNX 已查清
  WF_TRON1A + isaacgym 模型      █████████░  90%  policy/encoder 可加载
  /fcr_tron/cmd_vel override    ████████░░  80%  本地补丁存在，需整理到外部 repo
  controller watchdog            ███████░░░  70%  超时清零意图已补，零命令漂移待解
  遥控器 axes/buttons            ██████████ 100%  SensorJoy 已确认
  controller 激活                ███████░░░  70%  L1 + 三角/Y 可启动，但真机测试暂停

TRON1 仿真
  Gazebo / official sim launch   ████████░░  80%  可启动，rqt steering 默认关闭
  /fcr_tron/cmd_vel 控制链       ███████░░░  70%  可驱动，可归零
  基础动作                       ██████░░░░  60%  forward/back 可控，纯 yaw 有横移
  仿真安全验收                   ████████░░  80%  47/47 PASS；官方 Gazebo pose 漂移保留为 blocker

TRON1 真机
  PC <-> TRON Ethernet           ██████░░░░  60%  2026-09-03 曾通；本轮被 Mihomo 路由/ping BLOCK，需复核直连
  PC -> TRON SDK                 █████████░  90%  pointfoot_node 已连接
  物理 damping / hardware stop   ████████░░  80%  Motor in damping mode 已观察
  L1 + X 软件 stop               █████░░░░░  50%  已确认不是泄力/阻尼，后果仍需现场分步验收
  低速实机运动                   ███░░░░░░░  30%  能激活，但太猛，暂停

安全链路
  FCR limiter 限速               █████████░  90%  topic 输出层已验证
  input timeout -> zero          █████████░  90%  已验证
  shutdown zero burst            █████████░  90%  SIGINT/SIGTERM 尾包为 0
  kill/crash 兜底                █████░░░░░  50%  仍依赖 controller watchdog / 物理 stop
  裸 /cmd_vel 隔离               █████████░  90%  FCR 链路已隔离，真机前 gate 会拦截裸 TRON 订阅
  真机前 read-only gate          ████████░░  80%  A-10 逐项人工门 + live graph 显式声明已就位

集成目标
  Jetson 控 TRON                 █████░░░░░  50%  架构 ready；Jetson 实网/仿真安全/低速验收未完
  Jetson 控云台跟拍              █████████░  90%  已实测，可继续优化
  RS2 + Camera + TRON 协同       ████░░░░░░  40%  底盘未安全联调
  Full Person Following          ███░░░░░░░  30%  视觉/云台 ready，TRON 真机安全未过
```

## 当前推荐启动入口

### OpenCode 低风险辅助

第一次只需要登录 DeepSeek API key：

```bash
/home/miya/.opencode/bin/opencode auth login --provider deepseek
/home/miya/.opencode/bin/opencode auth list
/home/miya/.opencode/bin/opencode models deepseek
```

之后默认使用 DeepSeek V4 Flash：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./scripts/opencode-fallback.sh "只读检查当前项目状态，不启动 ROS，不接触真机运动。"
```

### TRON1 只读预检

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./tools/tron1_bringup/tron1_safety_acceptance_check.sh
./tools/tron1_bringup/tron1_real_motion_path_preflight.sh
```

`tron1_real_motion_path_preflight.sh` 是只读检查：它只看路由、ping、launch 默认值和 ROS graph，不会启动 controller 或发布速度。若 `10.192.1.2` 走 `Mihomo`/Meta/TUN/utun/tap/wg/container/policy table、ping 不通、路由接口不是有线形态，或默认安全参数不可确认，会返回非零并阻止进入实机准备。可设置 `TRON_LINK_IFACE=enp0s31f6` 这类显式白名单，强制只接受指定接口；退出码语义是 `0=PASS, 1=FAIL, 2=WARN-only, 3=BLOCK`。

### TRON1 实机分步验收路线

```text
1. PC 直连 TRON：PC 上跑 FCR 安全栈和键盘控制台，验证架空/支架低速短脉冲。
2. Jetson 直连 TRON：FCR 安全栈和键盘控制台都跑在 Jetson；PC 只通过 SSH 观察和操作。
3. Jetson 加云台/相机：Jetson 同时跑 RS2、Gemini/Sony 和 TRON 通信，只做低速底盘修正，不做全速跟拍。
```

详细步骤见 [docs/TRON1_REAL_TEST_STEP_CHECKLIST.md](docs/TRON1_REAL_TEST_STEP_CHECKLIST.md)。

### TRON1 遥控器只读监视

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
ros2 run limxsdk_lowlevel pf_sensorjoy_monitor 10.192.1.2
```

### TRON1 仿真优先

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

### Jetson 云台跟拍路线

```bash
cd /home/miya/follow_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bringup_pkg fcr_bringup.launch.py \
  use_sim:=false \
  enable_chassis:=false \
  enable_gimbal:=true \
  can_interface:=can1
```

## 安全原则

- 真机 TRON1 当前暂停运动；先回 Gazebo。
- Gazebo 只用于验证 launch、topic safety、limiter、timeout 和基础方向；`WF_TRON1A + isaacgym` pose 漂移尚未解决，不能替代真机低速安全验收。
- 不让 TRON1 订阅旧 `/cmd_vel`。
- 默认 `enable_motion=false`。
- `L1 + X` 不是泄力；物理 motor switch / hardware action 才观察到 damping。
- 即使 read-only gate 无 `FAIL/BLOCK`，结论也只是可进入架空/支架、`enable_motion=false` 起步、极低速短脉冲分步测试，不是 100% 安全保证，也不是地面自由运动许可。
- 下一次真机必须有人扶稳/架空或支架、空旷低速、物理急停可触达，并按 [docs/TRON1_REAL_TEST_STEP_CHECKLIST.md](docs/TRON1_REAL_TEST_STEP_CHECKLIST.md) 逐步放行。

## 未来优化方向

工程优先：

- 把 TRON1 官方 repo 的本地补丁整理成可复现 patch 或 fork。
- 先完成 TRON1 Safety Acceptance Checklist，不再以“能动”为目标。
- 在 Jetson 上验证 Jetson <-> TRON1 Ethernet 和 `/fcr_tron/cmd_vel` ROS graph。
- 在 Gazebo 中系统复核 stop、timeout、lost-command、node-crash、damping 行为。
- 将底盘控制抽象成更干净的 `base_interface`，只让 adapter/limiter 知道 TRON1 细节。
- 将云台负责短时小角度修正、TRON1 负责长期 yaw/距离补偿，避免双控制器互相抢误差。

值得发展成论文/毕业设计的方向：

- 云台-移动底盘协同视觉伺服：快慢双环、误差分配与稳定性分析。
- 面向双轮足机器人的安全速度屏障：速度/加速度限制、命令超时、目标丢失、急停状态机。
- 人物跟拍中的主动感知：云台保持目标居中，底盘优化距离和视角质量。
- 视觉跟踪不确定性驱动控制：检测置信度、深度置信度、遮挡状态进入控制权重。
- TRON1 双轮足跟拍的 sim-to-real 安全迁移：从 Gazebo/Isaac 到低速实机的 gate-based workflow。
- 多模态交互跟拍：遥控器、语音、视觉目标选择和安全模式切换的统一仲裁。

## 目录提示

```text
src/perception_pkg/       Sony/Orbbec 感知、检测、跟踪、深度融合
src/servo_control_pkg/    视觉伺服、跟拍控制、云台/底盘控制分配
src/robot_platform_pkg/   RS2 驱动、TRON1 safety limiter、平台状态
src/bringup_pkg/          一键启动 launch
tools/tron1_bringup/      Jetson/TRON 网络与安全预检脚本
docs/                     handoff、迁移、安全和测试记录
docs/ai/                  给 Codex/OpenCode 的项目上下文
```
