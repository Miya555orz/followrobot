# Codex Handoff Prompt — FCR ROS2 → TRON1 Migration

Date: 2026-09-01

请你作为资深 ROS 2/C++/机器人系统工程师，继续接手 Miya 的跟拍机器人迁移项目。用户 Linux/ROS2 经验较少，需要你给出非常具体、可复制、低风险的命令行步骤，并在涉及真机运动、刷机、磁盘、CAN 通信时优先保证安全。

## 项目目标

把学长的 `fcr_ros2_3` 跟拍机器人项目从原来的 LEKIWI 三全向轮底盘迁移到逐际动力 TRON1 EDU 双轮足平台。第一次实机链路暂定为：

- TRON1 EDU 双轮足底盘；
- Jetson Orin Nano CLB 开发者套件；
- RS2 云台，通过 CAN 或 USB-CAN 链路接入；
- Orbbec Gemini 335 深度相机；
- 暂时没有 Sony 相机，Sony 相关链路不要作为首测阻塞项。

## 当前仓库/工作区

- FCR 工作区：`/home/miya/follow_ws`
- FCR 源码：`/home/miya/follow_ws/src/fcr_ros2_3`
- GitHub：`https://github.com/Miya555orz/followrobot`
- TRON1 官方工作区：`/home/miya/limx_ws`
- TRON1 仿真机器人类型：`WF_TRON1A`
- ROS 2：Humble
- Ubuntu：22.04

## 已完成进展

1. `fcr_ros2_3` 已能在本机全量/核心编译。
2. 修复过 Orbbec SDK 共享库符号链接：
   - `libOrbbecSDK.so -> libOrbbecSDK.so.2`
   - `libOrbbecSDK.so.2 -> libOrbbecSDK.so.2.9.3`
3. Orbbec Gemini 335 已验证：
   - 需要 USB3，`lsusb -t` 应显示 `5000M`；
   - 曾因 Type-C 线/扩展坞链路只跑到 `480M` 导致深度帧率不稳；
   - 换线后 `/camera/depth/image_raw` 可稳定约 `10 Hz`。
4. TRON1 官方仿真可启动，Gazebo 中机器人能出现并运动。
5. 已确认 TRON1 仿真 `/cmd_vel` 类型为 `geometry_msgs/msg/Twist`。
6. 学长项目的速度链路中存在 `TwistStamped`，已经设计并实现 TRON1 安全限速适配层。
7. 已新增/调整：
   - `robot_platform_pkg` 中的 TRON1 safety limiter；
   - `teleop_control_pkg` 中的中文命令行控制器；
   - TRON1 外部控制器补丁说明；
   - 仿真测试框架文档；
   - Jetson + RS2 bring-up 文档。

## TRON1 安全限速链路

设计目标：禁止跟拍算法或手动测试直接高速写入 TRON1 `/cmd_vel`。

当前链路：

```text
中文 teleop / FCR 跟拍输出
  -> /fcr/cmd_vel_stamped     geometry_msgs/msg/TwistStamped
  -> tron1_safety_limiter
  -> /fcr_tron/cmd_vel        geometry_msgs/msg/Twist
  -> TRON1 仿真/控制器
```

核心安全策略：

- 限制 `linear.x`；
- 强制 `linear.y = 0`；
- 限制 `angular.z`；
- 输入超时自动停车；
- 支持 `/safety/estop_state` 急停；
- 初期测试速度应保持非常低，例如 `linear.x <= 0.10 m/s`、`angular.z <= 0.15 rad/s`。

常用仿真启动命令：

```bash
source /opt/ros/humble/setup.bash
source ~/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
ros2 launch robot_hw pointfoot_hw_sim.launch.py fcr_cmd_vel_topic:=/fcr_tron/cmd_vel
```

限速器：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=true \
  max_linear_x:=0.10 \
  max_angular_z:=0.15
```

中文控制：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 run teleop_control_pkg tron1_chinese_teleop --ros-args \
  -p default_linear_x:=0.08 \
  -p max_linear_x:=0.10
```

用户可输入：

```text
直走
后退
左转
右转
慢慢直走
快点左转
停
急停
解除急停
帮助
退出
```

已观察到：左右转响应明显；低速前后和停车在 Gazebo 中可能存在惯性/控制器状态影响。用户已经确认 `/cmd_vel` 输出会归零，因此“停了还滑一点”更像仿真动力学/控制器惯性，不是限速器继续发非零速度。

## Jetson Orin Nano CLB 当前状态

用户现在正在给 Jetson Orin Nano CLB 开发者套件刷 JetPack。

重要背景：

- SDK Manager GUI 在本机容易崩溃；
- CLI 可用；
- 当前 SDK Manager 版本：`2.4.1.13536`；
- 用户已经能让 Jetson 进入 Recovery/APX 模式；
- 主机可看到：

```bash
lsusb | grep -i nvidia
# 0955:7523 NVIDIA Corp. APX
```

非常重要：主机上执行 `lsblk` 看到的是笔记本自己的磁盘，不是 Jetson 的存储。不要把主机的 `sda` 或 `nvme0n1` 当成 Jetson 目标盘。

SDK Manager 第一次刷机失败的关键日志：

```text
--external-device sda1
External storage device (/dev/sda) might be unavailable
Flash failed: usb unavailable
Error Code: 524
```

判断：SDK Manager/用户选择把 Jetson 侧目标存储设成了 USB/`sda1`，但 Jetson initrd 环境中没有可用 `/dev/sda`，因此失败。不是电脑没识别 APX，也不是一定没电。

最新用户界面已经出现：

```text
Storage Device:
1. SD Card
2. NVMe
3. USB
4. Custom
```

已建议用户选择：

```text
2. NVMe
```

下一位 Codex 请继续从这里跟进。如果用户刷机继续失败，优先看最新 flash 日志，确认是否仍然是 `--external-device sda1`；正确 NVMe 路径应变为类似 `--external-device nvme0n1p1`。

## Jetson 刷机 CLI 命令记录

已用过的 SDK Manager CLI 路线：

```bash
sdkmanager --cli \
  --action install \
  --login-type devzone \
  --product Jetson \
  --target-os Linux \
  --version 6.2.3 \
  --show-all-versions \
  --target JETSON_ORIN_NANO_TARGETS \
  --install-method direct_flash \
  --select 'Jetson Linux' \
  --select 'Jetson Runtime Components' \
  --deselect 'Host SDK Components' \
  --deselect 'Jetson SDK Components' \
  --deselect 'Jetson Platform Services' \
  --flash \
  --license accept \
  --collect-usage-data disable
```

如果失败，诊断命令：

```bash
lsusb | grep -i nvidia
find ~/.nvsdkm/logs -type f | grep -i flash | tail -5
find ~/.nvsdkm/logs -type f | grep -i flash | tail -1 | xargs tail -120
grep -R "external-device" ~/.nvsdkm/logs | tail -20
```

## 下一步建议

1. 先完成 Jetson 刷机。
2. Jetson 首次开机后，在 Jetson 本机执行：

```bash
hostname
uname -a
cat /etc/nv_tegra_release
lsb_release -a
df -h
free -h
ip -brief address
```

3. 在 Jetson 上安装基础环境：

```bash
cd ~
git clone -b main https://github.com/Miya555orz/followrobot.git
cd ~/followrobot
bash tools/tron1_bringup/jetson_post_flash_setup.sh
```

4. 建立 ROS 工作区：

```bash
mkdir -p ~/follow_ws/src
cd ~/follow_ws/src
git clone -b main https://github.com/Miya555orz/followrobot.git fcr_ros2_3
source /opt/ros/humble/setup.bash
cd ~/follow_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
  --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg
```

5. Jetson + RS2 CAN 检查：

```bash
cd ~/follow_ws/src/fcr_ros2_3
bash tools/tron1_bringup/jetson_rs2_preflight.sh
ip -details link show
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg --ctime | tail -120
```

6. 如果出现 `can0`，再配置 CAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
candump can0
```

## 安全提醒

- 不要直接让跟拍算法控制 TRON1 真机 `/cmd_vel`。
- 真机首次移动必须经过 safety limiter。
- 初始速度建议低于 `0.10 m/s`，转向低于 `0.15 rad/s`。
- 真机测试前必须能确认：
  - 停车命令输出为零；
  - 超时自动停车有效；
  - 急停有效；
  - 没有旧的 `ros2 topic pub`、`rqt_robot_steering`、Gazebo/控制器残留。

## 给下一位 Codex 的工作方式

- 请先读代码/日志再判断，不要凭空猜。
- 优先使用 `rg`。
- 用户是初学者，命令要完整，少省略。
- 刷机/磁盘/真机运动相关操作要解释清楚风险。
- 如果需要用户操作实体板子，请说清楚“现在应该看到什么灯/什么输出/下一步做什么”。
- 每推进一个阶段，及时更新 `docs/jetson_orin_nano_clb_setup.md`、`docs/jetson_rs2_bringup_log.md` 或本交接文件。
