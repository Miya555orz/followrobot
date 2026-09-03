# TRON1 安全模式管理验收记录

日期：2026-09-03

结论：

```text
PASS：32/32 组通过。
PASS：带 --with-gazebo 时，脚本确认 ROS graph 中存在 Gazebo 或 robot_hw_node。
PASS：状态机未进入 TRON_FOLLOW 前，/fcr_tron/cmd_vel 始终保持 0。
PASS：进入 TRON_FOLLOW 后，输出仍被 limiter 限幅。
PASS：timeout、外部急停、软件 estop 都会让输出回到 0。
```

运行命令：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
source /home/miya/limx_ws/install/setup.bash

cd /home/miya/follow_ws/src/fcr_ros2_3
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh --with-gazebo
```

关键输出：

```text
[PASS] 01 初始/复位后进入 IDLE
[PASS] 02 IDLE 不授权运动
[PASS] 03 IDLE 下输入大速度仍输出零
[PASS] 04 非法跳转到 TRON_FOLLOW 被拒绝
[PASS] 05 非法跳转后仍不授权
[PASS] 06 进入开发者模式
[PASS] 07 开发者算法自检通过
[PASS] 08 进入同款蹲起/站立准备
[PASS] 09 进入同款行走准备
[PASS] 10 行走准备态默认仍不授权运动
[PASS] 11 行走准备态大速度仍输出零
[PASS] 12 设备自检通过
[PASS] 13 云台进入跟随模式
[PASS] 14 云台跟随态仍不授权 TRON 运动
[PASS] 15 进入 TRON 跟随态
[PASS] 16 TRON 跟随态授权运动
[PASS] 17 TRON 跟随态 linear.x 被限幅: max_x=0.0300
[PASS] 18 TRON 跟随态 linear.y 被强制为零: max_y=0.0000
[PASS] 19 TRON 跟随态 angular.z 被限幅: max_yaw=0.1000
[PASS] 20 输入 timeout 后输出自动归零
[PASS] 21 外部急停强制进入 ESTOP
[PASS] 22 外部急停后取消运动授权
[PASS] 23 外部急停后输出归零
[PASS] 24 急停锁存时拒绝继续进入 TRON_FOLLOW
[PASS] 25 外部急停未解除时 clear_estop 无效
[PASS] 26 外部急停未解除时仍不授权
[PASS] 27 clear_estop 后回到 IDLE
[PASS] 28 clear_estop 后仍不授权
[PASS] 29 软件模式请求 estop 进入 ESTOP
[PASS] 30 软件 estop 后不授权
[PASS] 31 reset 可回到 IDLE
[PASS] 32 reset 后输出保持零

结果：32/32 组通过
Gazebo/robot_hw_sim 已随测试启动；姿态漂移不作为本脚本判定项。
```

已验证的安全边界：

- `tron1_mode_manager_node` 默认 `IDLE`，不授权运动。
- 非法跳转无法直接进入 `TRON_FOLLOW`。
- `REMOTE_WALK_READY` 默认仍不授权运动，避免“同款行走准备”变成实际运动许可。
- 只有按顺序进入 `TRON_FOLLOW`，`/tron1/motion_authorized` 才为 true。
- `tron1_safety_limiter_node` 需要同时满足：
  - `enable_motion=true`
  - `/tron1/motion_authorized=true`
  - 没有急停
  - 输入未 timeout
- 横移 `linear.y` 被强制为 0。

未验证内容：

- 真实 TRON1 地面停止距离。
- 遥控器固件层面的急停解除。
- `0000E103` 官方错误码精确定义。
- 官方遥控器开发者模式裸 `/cmd_vel` 路径是否能被固件侧禁用。
