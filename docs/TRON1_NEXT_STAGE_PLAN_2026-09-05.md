# TRON1 下一阶段计划

日期：2026-09-05

目标：把第十五轮审查和 DeepSeek/OpenCode 建议转成可执行计划。本文不是实机运动许可；在 A-10、官方 controller/SDK/hardware stop 路径、Gazebo 零漂 blocker 和分步 checklist 全部处理清楚前，真实 TRON1 仍保持暂停运动。

## 立即采纳

这些项低风险，优先级最高，多数不需要机器人运动：

1. 称量上装总重并记录安装高度/重心趋势。
   - 记录 Jetson、RS2、Gemini、Sony、电池、支架和线缆总重。
   - 能下移的部件优先下移；不要把“起立太猛”先归因到 FCR 增益。
2. 打通 Jetson 到 TRON1 的以太网链路，只做 non-motion hardware check。
   - 允许开机、静态 IP、路由和 ping；不发布速度，不按 `L1 + 三角/Y`，不激活官方 `WheelfootController`。
   - 目标证据：Jetson 静态 IP、ping `10.192.1.2`、read-only preflight 输出。若需要启动官方 node 建立 SDK 连接，必须转入架空/支架 Step 1，并记录为 controller bring-up，不再称作 non-motion 网络检查。
   - 2026-09-05 当前 PC 侧只读 preflight 发现 `10.192.1.2` 走 `Mihomo`/policy table 2022 且 ping 不通；这是实机前置 `BLOCK`，需先恢复直连路由。
   - 尚未用网线直连 TRON1 时，route/ping `BLOCK` 是预期安全状态，不代表脚本或代码缺陷；接好直连网线后再重新跑只读 preflight。
3. 起立瞬态稳定后才允许 FCR 授权，先做规程版。
   - `L1 + 三角/Y` 激活官方 controller 后，不立刻打开 `enable_motion=true`。
   - 先人工等待并记录稳定时间 `N`，初始建议 `N=10s`，最终以第一次架空/支架 IMU 和现场观察为准。
   - 当前不写“自动稳定门”代码，因为仓库内还没有 TRON1 IMU/姿态状态输入可证明稳定。
4. 接受 software crash-stop 的边界。
   - `/fcr_tron/cmd_vel` 的软件路径只能把期望速度归零，不能证明物理停止/阻尼。
   - 真机分步测试必须有人握持遥控器或可立即触达物理停止/阻尼动作。
5. 停止新增同类门禁脚本。
   - 现有 47/47 自动验收、read-only A 门、graph 守卫和 A-10 人工门已经足够进入审查。
   - 后续新增代码前，优先换取只读硬件证据和架空/支架低速机时。

## 暂不直接采纳到运行链路

这些建议方向正确，但会改变真实底盘控制输出，必须先设计、单测和仿真审查：

1. 目标速度前馈。
   - 现状：`mvp_follow_controller_node` 的底盘距离控制仍是 `k_base_z * ez` 纯 P。
   - 风险：直接叠加观测 `velocity[2]` 可能坐标系/符号错误，或和底盘实际速度形成隐性正反馈。
   - 处理：先写设计与单测，确认相机系径向速度、base_link 前向、云台 yaw 和上一周期底盘命令之间的坐标关系；只在 VALID 新鲜深度下启用。
2. 限速分档。
   - 现状：TRON1 limiter 默认 `max_linear_x=0.03`、`max_angular_z=0.10`，只适合接线和短脉冲验证。
   - 处理：可以设计 L0-L3 参数表，但任何真机档位提升必须由 `TRON1_REAL_TEST_STEP_CHECKLIST.md` 人工逐级确认，脚本不得自动放宽。
3. 深度融合外推到消费端 `now()`。
   - 现状：depth fusion 预测到 `track_stamp`，不是控制消费时刻。
   - 处理：先量端到端延迟，并确认 `prediction_hold_s` 大于最大感知延迟；外推不得让 `PREDICTED` 或 stale depth 驱动底盘平移。
4. 躯干扰动补偿。
   - 处理：先做判据实验。TRON1 原地站立/低速行走时，云台 hold，拍静止目标，统计像素 RMS；`RMS < 3px` 则不写补偿，单峰振动再考虑 notch，低频漂移再考虑 IMU 前馈。

## 两周目标分级

| 级别 | 演示内容 | 当前态度 |
| --- | --- | --- |
| A 保底 | TRON1 站立/遥控移动，云台自主跟踪走动的人；底盘只作为载体或人工遥控 | 优先保证 |
| B 目标 | A + 底盘低速距离/yaw 修正，人在 deadman/物理停止旁，逐级限速 | 需要 Step 1/2 证据 |
| C 冲刺 | 全自主 2 m 跟拍、目标自由走动 | 暂不承诺 |

## 明天第一小时

```text
1. git status，确认 followrobot/main clean。
2. 不接 TRON1 运动链路，整理上装重量和安装高度记录表。
3. 准备 Jetson<->TRON1 non-motion 网络检查命令；该检查只到链路/IP/ping，不激活官方 controller。
4. 更新 A-10/Step 1 记录模板，加入起立稳定等待 N 秒和 IMU/现场观察栏。
5. 只读跑 gate；BLOCK 是预期安全状态，不把它当失败。
```

## 仍然禁止

- 不做地面自由运动。
- 不做 Sony + RS2 + TRON 全链路自动跟拍。
- 不让裸 `/cmd_vel` 成为 TRON1 入口。
- 不把 `PASS_FOR_STAGED_AIRBORNE_TEST` 写成 100% 安全或地面运动许可。
