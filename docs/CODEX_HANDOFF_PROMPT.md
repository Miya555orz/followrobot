# Copy-paste prompt for the next Codex session

你正在接手一个正在进行中的 `followrobot / fcr_ros2_3 -> TRON1 EDU` 迁移项目。以下内容是上一位 Codex 留下的工程状态快照。不要假设项目从零开始，也不要重复已经完成的排查。首先阅读本文、`docs/HANDOFF.md`、`README.md` 和当前仓库状态，再继续开发。

Active GitHub repo: <https://github.com/Miya555orz/followrobot>

Active local checkout:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
```

Target remote:

```text
followrobot  https://github.com/Miya555orz/followrobot.git
```

Upstream original remote:

```text
origin       https://github.com/cuiangA/fcr_ros2_3.git
```

不要把迁移工作推送到 `origin`，默认只推送到 `followrobot/main`。

## Mission

原项目 `fcr_ros2_3` 是学长留下的 ROS2 Humble 跟拍机器人项目，包含感知、目标跟踪、视觉伺服、DJI RS2 云台、命令仲裁、安全逻辑和旧三全向轮底盘代码。当前项目 `followrobot` 要把上层跟拍能力迁移到 LimX / 逐际动力 TRON1 EDU 双轮足底盘。

核心架构目标：

```text
Camera / Sensors
    -> Perception
    -> Tracking
    -> Follow / Visual Servo
    -> Command Mux + Safety
    -> Unified Base Interface
    -> Platform Adapter
       ├── Legacy Omni Adapter
       └── TRON1 Adapter -> TRON1 SDK / Ethernet -> TRON1 EDU
```

最重要约束：上层跟拍算法不应关心底盘类型。换底盘应尽量只换 adapter package / launch / config，而不是改感知和跟随业务代码。

## Verified state

- `[VERIFIED]` Jetson Orin Nano CLB 已刷好 JetPack 6.2.3 / Jetson Linux R36.5.2，并从 NVMe 启动。
- `[VERIFIED]` Jetson OS: Ubuntu 22.04.5 LTS, kernel `5.15.199-tegra`, aarch64。
- `[VERIFIED]` Jetson ROS2 Humble 可用：`/opt/ros/humble/bin/ros2`。
- `[VERIFIED]` 当前推荐 SSH 方式是以太网：

```bash
ssh miya@172.31.178.242
```

- `[VERIFIED]` PC 到 Jetson 的免密 SSH 已通过 `ssh-copy-id` 配置。
- `[VERIFIED]` Jetson 工作区 `~/follow_ws` 中可见：
  - `vision_servo_msgs`
  - `robot_platform_pkg`
  - `teleop_control_pkg`
  - `orbbec_camera`
  - `orbbec_camera_msgs`
  - `sony_camera_pkg`
- `[VERIFIED]` Jetson 板载 CAN 是 `can0`，但当前 RS2 不走它。
- `[VERIFIED]` 当前 RS2 实测链路是 external USB-CAN -> SocketCAN `can1` -> DJI RS2。
- `[VERIFIED]` `can1` 已经 `UP`、`ERROR-ACTIVE`、bitrate 1 Mbps、driver `gs_usb`。
- `[VERIFIED]` RS2 ROS2 driver 可启动：`/gimbal/status connected=true`，极小 yaw 指令已确认云台会动，CAN/CRC/parse errors 为 0。
- `[VERIFIED]` Orbbec Gemini 335 已在 USB3/5000M 下输出 `/camera/depth/image_raw`，424x240@10Hz。
- `[VERIFIED]` RS2 + Orbbec 共存测试通过。
- `[VERIFIED]` Sony ZV-E10M2 UVC 实流已验证：`/dev/video8` 可发布 `/sony/image_raw` 和 `/sony/camera_info`。
- `[VERIFIED]` `perception_pkg sony_uvc_perception.launch.py` 已跑通 YOLOv8n CPU detection/tracking/aim smoke test；画面中有人时能检测到 `person`、生成 track 和 `/perception/aim_target_2d`。
- `[VERIFIED]` Sony UVC 人像识别 + DJI RS2 保守中速闭环跟随已跑通：`/perception/aim_target_2d -> gimbal_visual_servo_node -> /auto/cmd_gimbal -> command_mux -> /cmd_gimbal -> gimbal_driver_node -> can1 -> RS2`。方向已确认正常，最终实验配置为 yaw `0.12 rad/s`、pitch `0.075 rad/s` 上限。
- `[VERIFIED]` 2026-09-02 收工时已停止所有相机/识别/云台跟随/viewer 进程。
- `[VERIFIED]` `tools/visualization/ros_image_mjpeg_viewer.py` 可在 `http://<JETSON_IP>:8088/` 显示 Sony 原始图和 OpenCV debug 图。
- `[UNVERIFIED]` Sony CRSDK 版 `sony_camera_node` 未验证；CRSDK 不在仓库中，节点当前未构建。
- `[PARTIAL]` TRON1 PC-side Ethernet and SDK connection were verified on 2026-09-03. `pointfoot_node` connected to `10.192.1.2`, loaded `WF_TRON1A` / `isaacgym`, and subscribed to `/fcr_tron/cmd_vel`.
- `[VERIFIED]` LimX SDK `SensorJoy` received remote axes/buttons. `L1 + Y/triangle` activated `WheelfootController`.
- `[VERIFIED]` Physical motor switch/hardware action produced `Motor in damping mode`; official node stopped the controller and exited.
- `[IMPORTANT]` `L1 + X` is software `stopController()` + `abort()`, not damping/torque release.
- `[PAUSED]` TRON1 real motion is paused because the first activation felt too strong/fast. Continue in Gazebo/simulation and remote-controller familiarization first. See `docs/TRON1_REMOTE_AND_SIM_SAFETY.md`.

## Start here

Local:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
git status --short
git remote -v
git log --oneline --decorate -8
```

Jetson:

```bash
ssh miya@172.31.178.242 '
hostname
lsb_release -ds
uname -a
cat /etc/nv_tegra_release
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 pkg list | grep -E "vision_servo_msgs|robot_platform_pkg|teleop_control_pkg|orbbec_camera$|orbbec_camera_msgs|sony_camera_pkg"
ip -brief addr
ip -details -statistics link show can1 || true
lsusb
lsusb -t
'
```

## Network warning

优先使用：

```bash
ssh miya@172.31.178.242
```

不要优先折腾 `192.168.55.1`。之前 USB gadget SSH 出现过：

```text
Permission denied
REMOTE HOST IDENTIFICATION HAS CHANGED
ip route get 192.168.55.1 -> via 198.18.0.2 dev Mihomo
```

判断结论：PC 没稳定枚举 Jetson USB 网卡，并且 Mihomo/Clash TUN 劫持了 `192.168.55.1` 路由。那不是密码错，也不一定是 Jetson 坏了。

## RS2 commands

On Jetson:

```bash
cd ~/follow_ws/src/fcr_ros2_3
sudo bash /home/miya/follow_ws/src/fcr_ros2_3/tools/can/setup_gimbal_can.sh
ip -details -statistics link show can1
```

Expected:

```text
can1 UP
can state ERROR-ACTIVE
bitrate 1000000
driver gs_usb
errors 0
```

Launch:

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch robot_platform_pkg gimbal_bringup.launch.py \
  use_sim:=false \
  can_interface:=can1 \
  control_mode:=incremental_position
```

Check:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 param get /gimbal_driver can_interface
ros2 topic echo /gimbal/status --once
```

Tiny test only if status is healthy:

```bash
timeout 2s ros2 topic pub -r 10 /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
  "{yaw_rate: -0.03, pitch_rate: 0.0, hold_yaw: false, hold_pitch: true}"

ros2 topic pub --once /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
  "{yaw_rate: 0.0, pitch_rate: 0.0, hold_yaw: true, hold_pitch: true}"
```

## Orbbec commands

```bash
lsusb | grep -i orbbec
lsusb -t
```

Launch low-load depth:

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch orbbec_camera gemini_330_series_low_cpu.launch.py \
  camera_name:=camera \
  enable_color:=false \
  enable_depth:=true \
  depth_width:=424 \
  depth_height:=240 \
  depth_fps:=10 \
  enable_left_ir:=false \
  enable_right_ir:=false \
  enable_point_cloud:=false \
  enable_accel:=false \
  enable_gyro:=false
```

Check:

```bash
ros2 topic list | grep camera
timeout 8s ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
```

## TRON1 hard safety rule

TRON1 很重。不要直接让旧 `/cmd_vel` 控制 TRON1。

Required real-motion path:

```text
/fcr/cmd_vel_stamped
    -> robot_platform_pkg/tron1_safety_limiter_node
    -> /fcr_tron/cmd_vel
    -> TRON1 official controller
```

First real-test limits:

```text
enable_motion=false
max_linear_x=0.03 m/s
max_angular_z=0.10 rad/s
enable_lateral=false
timeout≈0.30s
```

Local official TRON1 repos:

```text
/home/miya/limx_ws/src/tron1-rl-deploy-ros2
/home/miya/limx_ws/src/tron1-gazebo-ros2
/home/miya/limx_ws/src/limxsdk-lowlevel
```

`tron1-rl-deploy-ros2` has a local patch allowing the official controller to subscribe to `/fcr_tron/cmd_vel` through `FCR_TRON_CMD_VEL_TOPIC`.

## Immediate priorities

1. Re-check current git status; preserve user changes.
2. Re-run RS2 + Orbbec coexistence after fresh boot.
   - Acceptance: `/gimbal/status connected=true`, `can1 ERROR-ACTIVE`, `/camera/depth/image_raw` around 10Hz.
3. Clean up `can0`/`can1` docs and launch defaults carefully.
   - Current real RS2 bench path is `can1`.
   - Jetson board `can0` exists but is separate.
4. Design `base_interface` and adapter layer before large code edits.
5. Verify Jetson <-> TRON1 Ethernet topology from official SDK and real hardware, with no motor command at first.
6. Re-run TRON1 simulation + limiter before any real TRON1 motion.

Commit/push only to:

```bash
git push followrobot HEAD:main
```
