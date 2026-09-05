# 当前状态

最后对齐日期：2026-09-05。

## 距离下一阶段还有多远

- Jetson 控制 TRON1：约 60%。FCR 命令路径、历史 PC 侧 TRON Ethernet/SDK、47/47 组 Gazebo 安全验收、estop 样本新鲜度 fail-closed、真机误启动守卫都已经就位；2026-09-05 PC USB SSH 与 Jetson <-> TRON1 Ethernet network-only 检查已 PASS。这里的 PASS 只证明链路/IP/ping，不激活官方 controller；真实运动仍暂停，之后才进入架空/支架低速实机验收。
- Jetson 控制 RS2 云台跟拍：接近可用。Sony UVC -> perception/tracking -> RS2 over `can1` 已经在 Jetson 上跑通过；CRSDK 仍是可选且未验证。
- Jetson 控制 RS2 跟拍 + TRON1 底盘：还不能做真实全链路跟拍。TRON1 仍需保持在仿真和低速安全验收阶段，底盘只允许后续作为慢速 yaw / 距离修正，不应和云台抢同一个误差。
- FCR 运行时手柄左右摇杆急停覆盖关系：`[UNVERIFIED]`。官网称其为全局急停，但本地 ROS2 代码只证明 `L1 + Y` 启动 controller、`L1 + X` 停止 controller 并 abort，未证明左右摇杆按下会锁存并压过 FCR 连续 `/fcr_tron/cmd_vel`；详见 `docs/TRON1_FCR_REMOTE_ESTOP_SEMANTICS_2026-09-05.md`。

当前姿态：TRON1 真机运动暂停。继续以遥控器熟悉、Gazebo 仿真、安全链路和官方 controller 语义摸底为优先。

## 已验证

- Jetson + Sony 相机 + DJI RS2 云台可以协同运行。
- 视觉 -> tracking -> 云台控制可以完成真实 RS2 跟拍。
- 当前活跃仓库是 `/home/miya/follow_ws/src/fcr_ros2_3`。
- TRON1 官方工作区存在于 `/home/miya/limx_ws`。
- TRON1 ROS package 在本地 ROS 2 Humble 环境下可见。
- TRON1 官方 `robot_hw` launch 暴露 `fcr_cmd_vel_topic`；本地官方工作区仍包含让官方控制器订阅 `/fcr_tron/cmd_vel` 的 FCR override 补丁。
- FCR 侧 `robot_platform_pkg`、`teleop_control_pkg`、`bringup_pkg` 可以在本机成功 build。
- TRON1 侧 `robot_controllers` 和 `robot_hw` 可以在本机成功 build。
- TRON1 safety mode acceptance 已于 2026-09-04 通过 47/47 组 Gazebo/robot_hw_sim 验收；验收 Python 脚本默认使用独立 `ROS_DOMAIN_ID=83`，可由 `FCR_TRON_ACCEPTANCE_ROS_DOMAIN_ID` 或 `--ros-domain-id` 覆盖，拒绝空值/0/前导零/非十进制/越界值，并在发布验收速度前检查 `/fcr_tron/cmd_vel` graph，`--with-gazebo` 用独立 60 秒上限等待 `/gazebo` 节点。
- Gazebo 安全语义已冻结到 `docs/TRON1_GAZEBO_SAFETY_SEMANTICS_FREEZE_2026-09-05.md`：FCR limiter 输出层和 stale-command 归零是 PASS，`WF_TRON1A + isaacgym` zero-cmd pose drift 不是 PASS，继续作为 official sim-policy/physics blocker。
- TRON1 official sim launch 默认 `start_steering_gui=false`，FCR 安全链测试不会再自动启动 `rqt_robot_steering`。
- FCR 的“遥控/连续遥控”指电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄摇杆。Jetson 测试时，控制台应跑在 Jetson 上，PC 只通过 SSH 输入。
- TRON1 实机分步验收清单见 `docs/TRON1_REAL_TEST_STEP_CHECKLIST.md`。
- 未来实机测试 runbook 见 `docs/TRON1_REAL_TEST_RUNBOOK.md`；未验证步骤均标为 `[UNVERIFIED - DO NOT EXECUTE]`，不得从聊天里临时拼真实运动命令。
- 当前阶段冻结记录见 `docs/TRON1_PHASE_FREEZE_2026-09-05.md`。
- 当前 DJI RS2 桌面/Jetson bench 接线使用 external USB-CAN `can1`。
- Jetson USB SSH quickstart 已记录：`docs/ai/JETSON_USB_SSH_QUICKSTART_2026-09-05.md`。2026-09-05 观察到 `192.168.55.1` 会被 `Mihomo table 2022` 捕获；修正后走 `enx964a2d124484 src 192.168.55.100`，ping 成功。
- PC 侧 Jetson 网络预检脚本存在：`tools/tron1_bringup/pc_jetson_network_preflight.sh`。
- TRON1 只读实机运动路径预检脚本存在：`tools/tron1_bringup/tron1_real_motion_path_preflight.sh`；它会把代理/TUN/container/policy-table 路由、非有线形态接口、与可选 `TRON_LINK_IFACE` 不一致的接口，或 `TRON_IP` ping 不通判为 `BLOCK` 并返回非零。官方 `robot_hw` 默认 `/cmd_vel` 只打印 INFO，真机仍必须用 FCR override 到 `/fcr_tron/cmd_vel`。
- Jetson 侧 TRON1 network-only 预检脚本存在：`tools/tron1_bringup/jetson_tron1_network_preflight.sh`；只检查 Jetson 到 TRON1 的 route/ping，不启动 ROS、`robot_hw` 或 controller。
- Jetson 侧 TRON1 临时路由设置脚本存在：`tools/tron1_bringup/jetson_tron1_route_setup.sh`；默认 dry-run，本机已验证 dry-run、错误接口 BLOCK 与无确认 `--apply` BLOCK 分支；Jetson 上 `--apply` 尚未端到端验证，后续需人工设置 `CONFIRM_TRON1_ROUTE_SETUP=yes` 后执行，且只执行 `ip link`/`ip addr`/`ip route` 并复查 route/ping。
- 2026-09-05 Jetson 到 TRON1 network-only PASS 已记录：`docs/ai/TRON1_JETSON_NETWORK_PREFLIGHT_2026-09-05.md`；`10.192.1.2 dev enP8p1s0 src 10.192.1.200`，ping 成功，脚本 `PASS=2 WARN=0 BLOCK=0 FAIL=0`。
- 2026-09-05 Jetson 到 TRON1 可复现网络设置流程已记录：`docs/ai/TRON1_JETSON_NETWORK_SETUP_REPEATABLE_2026-09-05.md`。
- 2026-09-05 PC 直连 TRON1 只读结果已记录：`docs/ai/TRON1_PC_DIRECT_PREFLIGHT_2026-09-05.md`。
- 2026-09-05 下午计划已记录：`docs/ai/TRON1_AFTERNOON_RD_TEST_PLAN_2026-09-05.md`。
- TRON1 迁移 gate report 存在：`docs/tron1_migration_gate_report_2026-09-03.md`。
- 第十五轮建议整理成 `docs/TRON1_NEXT_STAGE_PLAN_2026-09-05.md`；目标速度前馈、限速分档和 depth fusion 外推暂不直接进入真机链路。

## 当前工作

TRON1 EDU 双轮足底盘二次开发和整机系统集成。

## 重要注意事项

- 官方 TRON1 ROS2 文档面向 ROS 2 Iron；当前本机使用 ROS 2 Humble。
- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2` 中有本地补丁，允许官方控制器订阅 `/fcr_tron/cmd_vel`。
- OpenCode fallback dry run 不允许真实 TRON1 运动。
- 当前 PC 网络曾出现 Mihomo/TUN 路由劫持；实机低速验收阶段不建议依赖 PC 和 Jetson 跨机器 DDS 分发键盘。
- TRON1 PC 侧 Ethernet 已于 2026-09-03 打通过：`10.192.1.2` 当时可通过 `enp0s31f6` 访问，官方 `pointfoot_node` 能连接真机；2026-09-05 重新直连后只读 preflight 已 PASS。随后用户断开 PC 与 TRON1，改为 PC 通过 USB/SSH 到 Jetson、Jetson 网线到 TRON1，Jetson network-only preflight 也已 PASS。
- 未接 TRON1 直连网线时，route/ping `BLOCK` 是预期安全状态，不代表脚本或代码缺陷；接线后仍只能先重跑只读 preflight。
- TRON1 遥控器 axes/buttons 已通过只读 SDK monitor 观察到。`L1 + Y/三角` 会激活 `WheelfootController`。
- 物理 motor switch / hardware action 曾触发 `Motor in damping mode`；随后官方 `pointfoot_node` 停止 controller 并退出。`L1 + X` 应视作软件 stop/abort，不是 damping/torque release。
- `/safety/estop_state` 是 FCR `command_mux` 聚合软件急停状态，不是物理 motor switch；`/tron1/limiter_clear_estop` 是同一受控 ROS_DOMAIN 内的 limiter 软件恢复入口，不能替代物理急停/阻尼，真机 limiter-only 部署应使用隔离 domain/namespace 或外层访问控制。
- 真机前 A-10 已拆成逐项人工门：物理停止可触达、damping 证据、`L1+X` 语义、controller watchdog 后果、Gazebo 零漂 blocker 和实机分步 checklist 都必须显式确认；单个旧 `A10_CONFIRMED=yes` 不再足够。
- 真机前 read-only gate 的 live bringup 进程需要 `TRON1_LIVE_BRINGUP_INTENDED=yes` 显式声明，否则仍按残留进程 BLOCK；声明后还必须继续通过 `/fcr_tron/cmd_vel` 唯一 limiter 发布者、官方型节点名订阅和裸 `/cmd_vel` 检查。Gazebo/robot_hw_sim/steering GUI/裸 topic pub 仍然 BLOCK；graph 节点名不能替代硬件连接现场确认。
- TRON1 第一次 controller 激活体感过猛，因此真实运动暂停。下一步仍应先走 Gazebo、遥控器熟悉和低速安全验收。
- 起立瞬态和 FCR 运动授权必须分开：`L1 + 三角/Y` 激活官方 controller 后，先保持 FCR `enable_motion=false` 并记录稳定等待 `N` 秒；当前没有 TRON1 IMU/姿态输入可自动证明稳定，不能把计时当作 100% 安全。
- `enable_motion=false` 不能约束官方 controller 自己的起立/进入 WALK；所以官方 controller 启动或激活不属于 non-motion 网络检查。
- 当前 TRON1 安全状态：FCR limiter 的限幅、加速度限制、输入 timeout、estop 样本新鲜度、clean-shutdown zero burst 已在 topic 输出层通过；官方 controller watchdog 会清理陈旧速度意图。但 `WF_TRON1A + isaacgym` Gazebo 仍有零命令漂移和纯 yaw 横移。此前轻量 controller/URDF/friction/isaaclab 实验没有解决并已撤回/恢复，应把它视作官方 sim-policy blocker，而不是 FCR limiter bug。
- 不要在 controller/SDK/hardware stop 行为弄清楚前运行真实 TRON1 自动跟拍。
- 不能把官网左右摇杆“全局急停”当作已验证的 FCR 运行时覆盖急停；在当前固件/当前 FCR launch path 上验证前，它只能作为待验证备份路径。
