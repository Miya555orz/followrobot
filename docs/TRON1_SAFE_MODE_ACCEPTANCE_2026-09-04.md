# TRON1 安全模式管理验收记录

日期：2026-09-04

结论：

```text
PASS：47/47 组通过。
PASS：带 --with-gazebo 时，脚本确认 /fcr_tron/cmd_vel 只有 limiter 一个发布者，且官方 robot_hw 订阅 limiter 输出。
PASS：状态机未进入 TRON_FOLLOW 前，/fcr_tron/cmd_vel 始终保持 0。
PASS：进入 TRON_FOLLOW 后，输出仍被 limiter 限幅。
PASS：timeout、FCR 聚合软件急停 `/safety/estop_state`、软件 estop 都会让输出回到 0。
PASS：mode manager 死亡后，limiter 因授权信号超时继续输出 0。
PASS：`/tron1/limiter_state` 能说明当前放行意图/阻塞原因。
PASS：自动验收启动前有真机进程/网络守卫和 `/fcr_tron/cmd_vel` 预扫描，避免仿真测试误驱动真实 TRON1。
PASS：启动后、发布任何验收速度前，会做 ROS graph 订阅者守卫，非 Gazebo 模式除 probe 外出现任何 `/fcr_tron/cmd_vel` 订阅者都会拒绝继续。
PASS：验收 Python 脚本默认使用独立 `ROS_DOMAIN_ID=83`，支持 `FCR_TRON_ACCEPTANCE_ROS_DOMAIN_ID` 或 `--ros-domain-id` 覆盖，并拒绝空值或 `0`，降低与真实 TRON1 graph 混跑风险。
```

运行命令：

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
source /home/miya/limx_ws/install/setup.bash

cd /home/miya/follow_ws/src/fcr_ros2_3
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh --with-gazebo
```

代码基线：

```text
验收脚本会在每次运行末尾打印 `HEAD=<当前提交>, dirty_files=<未提交文件数>`。
由于 git commit 不能在文件中自包含自己的 hash，真机前以终端验收输出的 HEAD 和 clean/dirty 状态为准。
```

关键输出：

```text
[PASS] N01 enable_motion=false 时非零输入仍输出零
[PASS] N02 allow_tron_follow_motion=false 时 TRON_FOLLOW 仍不授权
[PASS] 启动前 ROS graph 守卫：启动前 /fcr_tron/cmd_vel 无发布者/订阅者
[PASS] ROS graph 真机误启动守卫：graph 订阅者守卫通过
[PASS] 00 /fcr_tron/cmd_vel 只有 limiter 一个发布者
[PASS] 00b 官方 robot_hw 订阅 limiter 输出
[PASS] 01 初始/复位后进入 IDLE
[PASS] 02 IDLE 不授权运动
[PASS] 02b limiter 状态显示未授权阻塞
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
[PASS] 16b limiter 状态显示正在尝试放行限幅命令
[PASS] 17 TRON 跟随态 linear.x 被限幅: max_x=0.0300
[PASS] 18 TRON 跟随态 linear.y 被强制为零: max_y=0.0000
[PASS] 19 TRON 跟随态 angular.z 被限幅: max_yaw=0.1000
[PASS] 20 输入 timeout 后输出自动归零
[PASS] 20b limiter 状态显示输入 timeout
[PASS] 20c estop 样本超时后继续发命令仍输出零
[PASS] 20d limiter 状态显示 estop 样本超时
[PASS] 21 外部急停强制进入 ESTOP
[PASS] 22 外部急停后取消运动授权
[PASS] 22b limiter 状态显示急停锁存
[PASS] 23 外部急停后输出归零
[PASS] 24 急停锁存时拒绝继续进入 TRON_FOLLOW
[PASS] 25 外部急停未解除时 clear_estop 无效
[PASS] 26 外部急停未解除时仍不授权
[PASS] 27 clear_estop 后回到 IDLE
[PASS] 28 clear_estop 后仍不授权
[PASS] 29 软件模式请求 estop 进入 ESTOP
[PASS] 30 软件 estop 后不授权
[PASS] 31 软件 estop 锁存时 reset 不能清除急停
[PASS] 32 clear_estop 后回到 IDLE
[PASS] 33 clear_estop 后输出保持零
[PASS] 34 死亡测试前已进入 TRON_FOLLOW 授权态
[PASS] 35 杀死 mode manager 进程
[PASS] 36 mode manager 死亡后继续发命令仍因授权超时归零
[PASS] 37 mode manager 死亡后 limiter 状态显示授权超时

结果：47/47 组通过
代码状态：HEAD=<运行时当前提交>, dirty_files=<运行时未提交文件数>
Gazebo/robot_hw_sim 已随测试启动；姿态漂移不作为本脚本判定项。
```

已验证的安全边界：

- `tron1_mode_manager_node` 默认 `IDLE`，不授权运动。
- `enable_motion=false` 在运行时已用独立 limiter 负向用例验证：非零输入仍输出 0。
- `allow_tron_follow_motion=false` 在运行时已用独立 mode manager 负向用例验证：即使按顺序进入 `TRON_FOLLOW`，仍不授权。
- 非法跳转无法直接进入 `TRON_FOLLOW`。
- `REMOTE_WALK_READY` 默认仍不授权运动，避免“同款行走准备”变成实际运动许可。
- 只有按顺序进入 `TRON_FOLLOW`，`/tron1/motion_authorized` 才为 true。
- `allow_tron_follow_motion` 默认 false；验收脚本会显式传 true，真机 launch 不会默认放行。
- `tron1_safety_limiter_node` 会检查授权信号新鲜度，mode manager 死亡后 0.5 秒内关门。
- `tron1_safety_limiter_node` 自己锁存急停；`/safety/estop_state=false` 不会自动解除 limiter 急停。
- `tron1_safety_limiter_node` 对 `/safety/estop_state` 做新鲜度检查；没样本或样本超时按急停处理。
- `/safety/estop_state` 是 FCR `command_mux` 聚合出的软件急停状态，不是 TRON1 物理 motor switch；`/tron1/limiter_clear_estop` 是同一受控 ROS_DOMAIN 内的 limiter 软件恢复入口，不能替代物理急停/阻尼。
- `tron1_safety_limiter_node` 发布 `/tron1/limiter_state`，用于区分 `INTENT_PASSING_LIMITED_CMD`、`BLOCKED_ESTOP_LATCHED`、`BLOCKED_ESTOP_TIMEOUT`、`BLOCKED_INPUT_TIMEOUT`、`BLOCKED_AUTHORIZATION_TIMEOUT` 等原因。
- `tron1_safety_limiter_node` 需要同时满足：
  - `enable_motion=true`
  - `/tron1/motion_authorized=true`
  - 授权信号未 timeout
  - 急停源样本新鲜且没有急停
  - 输入未 timeout
- 横移 `linear.y` 被强制为 0。

未验证内容：

- 真实 TRON1 地面停止距离。
- 遥控器固件层面的急停解除。
- `0000E103` 官方错误码精确定义。
- 官方遥控器开发者模式裸 `/cmd_vel` 路径是否能被固件侧禁用。
