# 当前状态

最后对齐日期：2026-09-04。

## 距离下一阶段还有多远

- Jetson 控制 TRON1：大约完成一半。FCR 命令路径、PC 侧 TRON Ethernet/SDK、47/47 组 Gazebo 安全验收、estop 样本新鲜度 fail-closed、真机误启动守卫都已经就位；还缺 Jetson <-> TRON1 Ethernet、Jetson 侧 ROS graph 检查、低速实机验收。
- Jetson 控制 RS2 云台跟拍：接近可用。Sony UVC -> perception/tracking -> RS2 over `can1` 已经在 Jetson 上跑通过；CRSDK 仍是可选且未验证。
- Jetson 控制 RS2 跟拍 + TRON1 底盘：还不能做真实全链路跟拍。TRON1 仍需保持在仿真和低速安全验收阶段，底盘只允许后续作为慢速 yaw / 距离修正，不应和云台抢同一个误差。

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
- TRON1 safety mode acceptance 已于 2026-09-04 通过 47/47 组 Gazebo/robot_hw_sim 验收；脚本强制独立 `ROS_DOMAIN_ID=83`，拒绝空值/0，并在发布验收速度前检查 `/fcr_tron/cmd_vel` graph。
- TRON1 official sim launch 默认 `start_steering_gui=false`，FCR 安全链测试不会再自动启动 `rqt_robot_steering`。
- FCR 的“遥控/连续遥控”指电脑键盘控制台 `fcr_mode_console`，不是 TRON 手柄摇杆。Jetson 测试时，控制台应跑在 Jetson 上，PC 只通过 SSH 输入。
- TRON1 实机分步验收清单见 `docs/TRON1_REAL_TEST_STEP_CHECKLIST.md`。
- 当前 DJI RS2 桌面/Jetson bench 接线使用 external USB-CAN `can1`。
- PC 侧 Jetson 网络预检脚本存在：`tools/tron1_bringup/pc_jetson_network_preflight.sh`。
- TRON1 只读实机运动路径预检脚本存在：`tools/tron1_bringup/tron1_real_motion_path_preflight.sh`。
- TRON1 迁移 gate report 存在：`docs/tron1_migration_gate_report_2026-09-03.md`。

## 当前工作

TRON1 EDU 双轮足底盘二次开发和整机系统集成。

## 重要注意事项

- 官方 TRON1 ROS2 文档面向 ROS 2 Iron；当前本机使用 ROS 2 Humble。
- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2` 中有本地补丁，允许官方控制器订阅 `/fcr_tron/cmd_vel`。
- OpenCode fallback dry run 不允许真实 TRON1 运动。
- 当前 PC 网络曾出现 Mihomo/TUN 路由劫持；实机低速验收阶段不建议依赖 PC 和 Jetson 跨机器 DDS 分发键盘。
- TRON1 PC 侧 Ethernet 已于 2026-09-03 打通：`10.192.1.2` 可通过 `enp0s31f6` 访问，官方 `pointfoot_node` 能连接真机。
- TRON1 遥控器 axes/buttons 已通过只读 SDK monitor 观察到。`L1 + Y/三角` 会激活 `WheelfootController`。
- 物理 motor switch / hardware action 曾触发 `Motor in damping mode`；随后官方 `pointfoot_node` 停止 controller 并退出。`L1 + X` 应视作软件 stop/abort，不是 damping/torque release。
- TRON1 第一次 controller 激活体感过猛，因此真实运动暂停。下一步仍应先走 Gazebo、遥控器熟悉和低速安全验收。
- 当前 TRON1 安全状态：FCR limiter 的限幅、加速度限制、输入 timeout、estop 样本新鲜度、clean-shutdown zero burst 已在 topic 输出层通过；官方 controller watchdog 会清理陈旧速度意图。但 `WF_TRON1A + isaacgym` Gazebo 仍有零命令漂移和纯 yaw 横移。此前轻量 controller/URDF/friction/isaaclab 实验没有解决并已撤回/恢复，应把它视作官方 sim-policy blocker，而不是 FCR limiter bug。
- 不要在 controller/SDK/hardware stop 行为弄清楚前运行真实 TRON1 自动跟拍。
