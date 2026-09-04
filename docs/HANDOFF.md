# followrobot / fcr_ros2_3 -> TRON1 EDU 工程交接文档

最后更新：2026-09-04，Asia/Shanghai

当前活跃 GitHub 仓库：<https://github.com/Miya555orz/followrobot>

这是当前 TRON1 迁移阶段的工程交接记录，用来保存代码状态、硬件连接、网络/SSH 坑、ROS2 启动路径、已测结果和已知风险。凡是没有在当前 workspace 或当前硬件会话中验证过的内容，都会显式标注。

## 标记说明

- `[VERIFIED]`：已经从当前 git/workspace、Jetson shell 或成功硬件测试中确认。
- `[UNVERIFIED]`：已经实现或推测可行，但尚未在当前真实硬件上验证。
- `[CONTEXT ONLY / NEEDS RE-VERIFY]`：来自前序对话、旧文档或旧日志，使用前需要重新确认。

## 1. 项目背景和最终目标

- `[VERIFIED]` `fcr_ros2_3` is the inherited ROS2 Humble follow-robot project. It contains perception, tracking, visual servoing, DJI RS2 support, command mux, safety logic, legacy base/platform code, and launch files.
- `[VERIFIED]` `followrobot` is Miya's active fork/repository for this migration: <https://github.com/Miya555orz/followrobot>.
- `[VERIFIED]` The upstream original remote is still present as `origin=https://github.com/cuiangA/fcr_ros2_3.git`; do not push migration work there unless explicitly asked.
- `[VERIFIED]` The target base is LimX / 逐际动力 TRON1 EDU. The original base was a LEKIWI / three-omni-wheel style base.

The migration is not just "replace a motor driver". The goal is to decouple the follow stack from the physical chassis:

```text
Camera / Sensors
    ↓
Perception
    ↓
Tracking
    ↓
Follow / Visual Servo
    ↓
Command Mux + Safety
    ↓
Unified Base Interface
    ↓
Platform Adapter
    ├── Legacy Omni Adapter -> old omni base
    └── TRON1 Adapter       -> TRON1 SDK / Ethernet -> TRON1 EDU
```

Design intent:

- Freeze the upper-level perception / tracking / follow code as much as possible.
- 底盘专用行为必须放在 adapter package、launch 和 config 中。
- 切换机器人底盘时，尽量不改上层业务逻辑。
- 绝不允许旧 FCR `/cmd_vel` 直接控制 TRON1 真机。
- 第一次 TRON1 真机测试必须经过 safety limiter，并使用极低速度、timeout 停止、急停和显式运动许可。

硬件角色：

- `[VERIFIED]` Jetson Orin Nano CLB：无头上车计算机，运行 ROS2 Humble、RS2 driver、Orbbec driver、跟拍/控制节点，以及后续 TRON1 adapter。
- `[VERIFIED]` DJI RS2：主动视觉云台，通过 Jetson external USB-CAN 控制。
- `[VERIFIED]` Orbbec Gemini 335：深度相机，目前已验证 depth-only 424x240@10Hz。
- `[VERIFIED]` Sony ZV-E10M2：UVC 模式枚举为 `/dev/video8`，发布 `/sony/image_raw`，已通过 YOLO/person detection、tracking、aim-target smoke test 和保守中速 RS2 闭环跟拍；专有 CRSDK node 仍未接入。
- `[VERIFIED]` 2026-09-02 Sony->RS2 实验室最终 profile 使用 `gimbal_visual_servo_low_speed_lab.yaml`；方向确认正常后，yaw 限制 `0.12 rad/s`，pitch 限制 `0.075 rad/s`。当天结束时相关实验进程均已停止。
- `[VERIFIED]` 轻量 live viewer：`tools/visualization/ros_image_mjpeg_viewer.py` 可在 `http://<JETSON_IP>:8088/` 提供 Sony raw 和 OpenCV debug 图像。
- `[PARTIAL]` TRON1 EDU：目标机器人底盘。2026-09-03 已观察到 PC Ethernet、SDK 连接、遥控器输入和 controller 激活，但由于首次激活体感过猛，真实运动暂停；后续先继续仿真。

## 2. Current code and directories

当前 ROS workspace：

```text
[VERIFIED] /home/miya/follow_ws
├── src/fcr_ros2_3/          # tracked project checkout
├── build/                   # generated, ignored
├── install/                 # generated, ignored
└── log/                     # generated, ignored
```

Active checkout:

```text
[VERIFIED] /home/miya/follow_ws/src/fcr_ros2_3
├── README.md
├── docs/
│   ├── HANDOFF.md
│   ├── CODEX_HANDOFF_PROMPT.md
│   ├── jetson_rs2_bringup_log.md
│   ├── jetson_rs2_depth_test_plan_2026-09-02.md
│   ├── rs2_gimbal_bringup.md
│   ├── tron1_jetson_comm_bringup.md
│   ├── tron1_migration.md
│   ├── tron1_external_repo_note.md
│   ├── tron1_sim_test_framework.md
│   ├── tron1_sim_test_log.csv
│   └── figures/system_architecture.svg|png
├── src/
│   ├── vision_servo_msgs/
│   ├── robot_platform_pkg/
│   ├── teleop_control_pkg/
│   ├── servo_control_pkg/
│   ├── perception_pkg/
│   ├── bringup_pkg/
│   ├── orbbec_camera/
│   ├── orbbec_camera_msgs/
│   ├── sony_camera_pkg/
│   ├── simulation_pkg/
│   ├── external_control_pkg/
│   ├── remote_monitor_pkg/
│   └── voice_intent_pkg/
└── tools/
    ├── can/setup_gimbal_can.sh
    └── tron1_bringup/
        ├── install_jetson_camera_deps.sh
        ├── jetson_post_flash_setup.sh
        ├── jetson_rs2_preflight.sh
        ├── pc_jetson_network_preflight.sh
        ├── tron1_real_motion_path_preflight.sh
        ├── rs2_can_preflight.sh
        ├── setup_jetson_mttcan_can0.sh
        ├── depth_camera_preflight.sh
        └── ros2_comm_snapshot.sh
```

Git remotes:

```text
[VERIFIED] followrobot https://github.com/Miya555orz/followrobot.git
[VERIFIED] origin      https://github.com/cuiangA/fcr_ros2_3.git
```

Do not upload:

- `[VERIFIED]` `build/`, `install/`, `log/`: local generated ROS build products.
- `[VERIFIED]` `src/sony_camera_pkg/sdk/`: proprietary Sony CRSDK staging area; ignored by git.
- Large model/runtime artifacts unless a future task explicitly decides how to version them.

Local TRON1 official code:

```text
[VERIFIED] /home/miya/limx_ws/src
├── tron1-rl-deploy-ros2/    # official TRON1 ROS2 controller, locally patched for FCR topic override
├── tron1-gazebo-ros2/       # official Gazebo sim repo
└── limxsdk-lowlevel/        # official low-level SDK/examples
```

## 3. Current software/hardware environment

Jetson facts checked over Ethernet SSH on 2026-09-02:

| Item | State |
|---|---|
| Hostname | `[VERIFIED]` `ubuntu` |
| OS | `[VERIFIED]` Ubuntu 22.04.5 LTS |
| Kernel | `[VERIFIED]` `5.15.199-tegra` aarch64 |
| Jetson Linux | `[VERIFIED]` R36, revision 5.2 |
| JetPack | `[VERIFIED]` 6.2.3 / R36.5.2 from bring-up context and `/etc/nv_tegra_release` |
| Rootfs | `[VERIFIED]` NVMe root was previously checked as `/dev/nvme0n1p1` |
| ROS2 | `[VERIFIED]` Humble at `/opt/ros/humble/bin/ros2` |
| Python | `[VERIFIED]` 3.10.12 |
| CMake | `[VERIFIED]` 3.22.1 |
| GCC | `[VERIFIED]` 11.4.0 |

Jetson interfaces:

| Interface | Meaning | State |
|---|---|---|
| `enP8p1s0` | Ethernet | `[VERIFIED]` UP, `172.31.178.242/24` |
| `l4tbr0` | USB gadget bridge | `[VERIFIED]` currently DOWN, owns `192.168.55.1/24` |
| `usb0`, `usb1` | USB gadget networking | `[VERIFIED]` currently DOWN |
| `can0` | Jetson board mttcan | `[VERIFIED]` exists but DOWN; not current RS2 path |
| `can1` | external USB-CAN | `[VERIFIED]` UP, ERROR-ACTIVE, 1 Mbps, driver `gs_usb` |

Currently visible USB devices on Jetson:

- `[VERIFIED]` `2bc5:0800 Orbbec Gemini 335`, USB tree shows `5000M`.
- `[VERIFIED]` `1d50:606f OpenMoko Geschwister Schneider CAN adapter`, USB tree shows driver `gs_usb`.
- `[VERIFIED]` Sony ZV-E10M2 was later confirmed as `054c:0ee8`, with usable UVC stream `/dev/video8` at 1280x720.

ROS2 packages visible after sourcing `/opt/ros/humble/setup.bash` and `~/follow_ws/install/setup.bash`:

```text
[VERIFIED] orbbec_camera
[VERIFIED] orbbec_camera_msgs
[VERIFIED] robot_platform_pkg
[VERIFIED] sony_camera_pkg
[VERIFIED] teleop_control_pkg
[VERIFIED] vision_servo_msgs
```

## 4. Network and SSH

### Recommended current path: Ethernet SSH

Current working topology:

```text
PC / laptop
    │ Ethernet / lab network
    ▼
Jetson Orin Nano CLB
    enP8p1s0 = 172.31.178.242/24
```

Use:

```bash
ssh miya@172.31.178.242
```

Passwordless SSH was configured from the PC:

```bash
ssh-keygen -t ed25519 -C "miya@Miya-to-jetson"
ssh-copy-id miya@172.31.178.242
ssh miya@172.31.178.242 'hostname; date; echo SSH_OK'
```

Expected:

```text
ubuntu
SSH_OK
```

### USB gadget SSH problem: do not repeat this loop

Previously, USB SSH to `192.168.55.1` produced:

```text
Permission denied
REMOTE HOST IDENTIFICATION HAS CHANGED
```

Critical diagnostic:

```bash
ip route get 192.168.55.1
```

Bad output seen before:

```text
192.168.55.1 via 198.18.0.2 dev Mihomo table 2022 src 198.18.0.1
```

Conclusion:

- `[VERIFIED]` The PC often did not enumerate the Jetson USB gadget NIC.
- `[VERIFIED]` Mihomo/Clash TUN captured the `192.168.55.1` route, so SSH was not reliably reaching the Jetson.
- `[VERIFIED]` Password failures on `192.168.55.1` were network/routing symptoms, not proof that the Jetson password changed.

If USB gadget must be retried:

```bash
ip -brief addr | grep -E 'usb|enx|192.168.55' || echo "No Jetson USB NIC"
ip route get 192.168.55.1
```

Only if a real `enx...` Jetson USB NIC is present:

```bash
JETSON_USB_NIC=enxREPLACE_WITH_ACTUAL_NAME
sudo ip addr add 192.168.55.100/24 dev "$JETSON_USB_NIC" 2>/dev/null || true
sudo ip link set "$JETSON_USB_NIC" up
sudo ip route del 192.168.55.0/24 2>/dev/null || true
sudo ip route add 192.168.55.0/24 dev "$JETSON_USB_NIC" metric 1
ip route get 192.168.55.1
ping -c 2 192.168.55.1
ssh miya@192.168.55.1
```

If host key is stale:

```bash
ssh-keygen -f ~/.ssh/known_hosts -R 192.168.55.1
```

### Proxy / apt

`[VERIFIED]` Campus network/captive portal previously broke `apt update` with `NOSPLIT` and unsigned InRelease errors. A working workaround was to reverse-proxy the laptop Clash/Mihomo port to the Jetson:

On PC:

```bash
ssh -fN -R 7892:127.0.0.1:7897 miya@172.31.178.242
```

On Jetson:

```bash
sudo apt update \
  -o Acquire::http::Proxy="http://127.0.0.1:7892" \
  -o Acquire::https::Proxy="http://127.0.0.1:7892"
```

If `apt` says release files are "not valid yet", check Jetson time:

```bash
date -R
timedatectl
```

## 5. Hardware topology

Current verified bench topology:

```text
                       Ethernet / SSH
PC / laptop ─────────────────────────────────▶ Jetson Orin Nano CLB
                                                    │
                                                    ├── USB3 ── Orbbec Gemini 335
                                                    │          └── ROS2: orbbec_camera
                                                    │
                                                    ├── USB ── USB-CAN adapter
                                                    │         └── SocketCAN can1 / gs_usb
                                                    │             └── CAN_H/CAN_L ── DJI RS2
                                                    │
                                                    ├── [UNVERIFIED] USB ── Sony camera
                                                    │
                                                    └── [UNVERIFIED] Ethernet ── TRON1 EDU controller
```

Important:

- `[VERIFIED]` Current RS2 uses external USB-CAN `can1`.
- `[VERIFIED]` Jetson board `can0` exists but is not the current RS2 wiring path.

## 6. Device startup manuals

### Jetson

```bash
ssh miya@172.31.178.242
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 topic list
```

Sanity check:

```bash
hostname
cat /etc/nv_tegra_release
which ros2
ros2 pkg list | grep -E 'vision_servo_msgs|robot_platform_pkg|teleop_control_pkg|orbbec_camera|orbbec_camera_msgs|sony_camera_pkg'
```

### DJI RS2

Connection:

```text
Jetson USB -> gs_usb USB-CAN -> CAN_H/CAN_L -> DJI RS2
```

Bring up CAN:

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
gs_usb
errors 0
```

Launch driver:

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch robot_platform_pkg gimbal_bringup.launch.py \
  use_sim:=false \
  can_interface:=can1 \
  control_mode:=incremental_position
```

Verify:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 param get /gimbal_driver can_interface
ros2 topic echo /gimbal/status --once
ip -details -statistics link show can1
```

Tiny yaw test only after `connected: true`:

```bash
timeout 2s ros2 topic pub -r 10 /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
  "{yaw_rate: -0.03, pitch_rate: 0.0, hold_yaw: false, hold_pitch: true}"

ros2 topic pub --once /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
  "{yaw_rate: 0.0, pitch_rate: 0.0, hold_yaw: true, hold_pitch: true}"
```

Verified result:

- `[VERIFIED]` RS2 moved by a very small yaw increment.
- `[VERIFIED]` `/gimbal/status connected=true`.
- `[VERIFIED]` CAN, CRC, and parse error counters stayed 0.

### Orbbec Gemini 335

Check:

```bash
lsusb | grep -i orbbec
lsusb -t
```

Expected:

```text
2bc5:0800 Orbbec Gemini 335
Driver=uvcvideo, 5000M
```

Launch depth-only low-load stream:

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

Verify:

```bash
ros2 topic list | grep camera
timeout 8s ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
```

Verified result:

- `[VERIFIED]` `/camera/depth/image_raw` was about 10 Hz.
- `[VERIFIED]` `/camera/depth/camera_info` was published.
- `[VERIFIED]` USB3 / 5000M observed.

### Sony camera

Current state:

- `[VERIFIED]` Source package `src/sony_camera_pkg` exists.
- `[VERIFIED]` CRSDK staging path is `src/sony_camera_pkg/sdk/` and is ignored by git.
- `[VERIFIED]` `sony_camera_node` is not currently built because CRSDK is not staged.
- `[VERIFIED]` Sony UVC `/dev/video8` image stream is verified; `/dev/video9` did not open and appears auxiliary.
- `[VERIFIED]` `perception_pkg/scripts/video_publisher.py --device /dev/video8` can publish `/sony/image_raw` and `/sony/camera_info`.
- `[VERIFIED]` `perception_pkg/launch/sony_uvc_perception.launch.py` starts the UVC publisher plus the existing detection/tracking/aim pipeline.

Check later:

```bash
lsusb
v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl not installed"
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 pkg executables sony_camera_pkg
```

UVC perception smoke-test:

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch perception_pkg sony_uvc_perception.launch.py \
  video_device:=/dev/video8 \
  width:=1280 \
  height:=720 \
  fps:=15 \
  inference_device:=cpu \
  confidence_threshold:=0.10 \
  model_path:=/home/miya/follow_ws/src/fcr_ros2_3/src/perception_pkg/models/yolov8n.onnx
```

Verified result:

- `[VERIFIED]` `/sony/image_raw` publishes at about 23 Hz.
- `[VERIFIED]` YOLOv8n ONNX loads and runs about 4.3-5.0 FPS on CPU.
- `[VERIFIED]` With a person in the frame, `/perception/detections` reports `person` with best confidence around 0.896.
- `[VERIFIED]` `/perception/tracks` creates tracking id `0`.
- `[VERIFIED]` `/perception/aim_target_2d` publishes `valid=true`.

### TRON1 EDU

Current state:

- `[VERIFIED]` Official repos exist under `/home/miya/limx_ws/src`.
- `[VERIFIED]` `tron1-rl-deploy-ros2` has a local topic override patch so official controller can subscribe to `/fcr_tron/cmd_vel`.
- `[UNVERIFIED]` Real Jetson <-> TRON1 Ethernet connection is not verified.
- `[CONTEXT ONLY / NEEDS RE-VERIFY]` Official diagrams/code suggest robot controller IP may be `10.192.1.2`; development computer/Jetson side may be `10.192.1.200`.

Required safe command path:

```text
/fcr/cmd_vel_stamped
    -> tron1_safety_limiter_node
    -> /fcr_tron/cmd_vel
    -> TRON1 official controller
```

First real-test limits:

```text
enable_motion=false until explicitly enabled
max_linear_x=0.03 m/s
max_angular_z=0.10 rad/s
enable_lateral=false
timeout ~= 0.30 s
```

Do not send real motor commands until TRON1 is physically protected and e-stop behavior is verified.

## 7. ROS2 system architecture

Key packages:

| Package | Responsibility | Status |
|---|---|---|
| `vision_servo_msgs` | shared custom msg/srv/action types, including `GimbalCmd`, `GimbalStatus`, `Target`, `TargetArray`, `PlatformState` | `[VERIFIED]` |
| `robot_platform_pkg` | RS2 driver, legacy chassis/platform code, `tron1_safety_limiter_node` | `[VERIFIED]` RS2; `[UNVERIFIED]` real base |
| `teleop_control_pkg` | command mux, keyboard teleop, deadman/e-stop/mode logic | `[VERIFIED]` source/package |
| `servo_control_pkg` | visual servo, PBVS/IBVS, velocity commander, follow logic | `[VERIFIED]` source; full real loop `[UNVERIFIED]` |
| `perception_pkg` | detection/tracking/depth fusion/Sony-Gemini calibration; Sony UVC fallback publisher | `[VERIFIED]` Sony UVC image + YOLO/tracking/aim smoke test |
| `bringup_pkg` | launch orchestration for FCR, RS2, TRON1 bridge | `[VERIFIED]` |
| `orbbec_camera` | Orbbec ROS2 camera driver | `[VERIFIED]` depth stream |
| `sony_camera_pkg` | Sony CRSDK camera wrapper | `[UNVERIFIED]` CRSDK node/stream |

Important topics:

| Topic | Message | Meaning |
|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Orbbec depth stream |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | depth camera calibration |
| `/cmd_gimbal` | `vision_servo_msgs/GimbalCmd` | final command to RS2 driver |
| `/gimbal/status` | `vision_servo_msgs/GimbalStatus` | RS2 health/state |
| `teleop/cmd_vel`, `auto/cmd_vel` | FCR base command inputs | pre-mux manual/auto path |
| `/cmd_vel` | legacy FCR mux output | do not directly connect to TRON1 |
| `/fcr/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | FCR safe input to TRON1 limiter |
| `/fcr_tron/cmd_vel` | `geometry_msgs/Twist` | limited TRON1 command topic |
| `/safety/estop_state` | `std_msgs/Bool` | FCR aggregated software e-stop gate; not the physical motor switch |

Current `can0`/`can1` rule:

- `[VERIFIED]` Current RS2 bench setup uses `can1`.
- `[VERIFIED]` TRON1/FCR communication launch defaults have been aligned to `can1` in `fcr_tron_full_follow.launch.py` and `fcr_tron_jetson_comm.launch.py`.
- `[VERIFIED]` Jetson board `can0` remains documented separately; only override back to `can0` after `ip link`/wiring verification.

## 8. Current progress checklist

```text
[✓] Original project/package architecture inspected.
[✓] GitHub fork `Miya555orz/followrobot` is active.
[✓] Jetson flashed and booted from NVMe.
[✓] Jetson ROS2 Humble environment works.
[✓] Core ROS2 packages visible after sourcing install.
[✓] External USB-CAN `can1` works with gs_usb.
[✓] RS2 ROS2 driver connects on can1.
[✓] RS2 tiny yaw control verified.
[✓] Orbbec Gemini 335 depth-only stream verified at about 10 Hz.
[✓] RS2 + Orbbec coexistence verified.
[✓] TRON1 simulation and limiter path re-checked locally on 2026-09-03.
[✓] TRON1 official controller topic override patch exists locally and exposes `fcr_cmd_vel_topic`.
[✓] TRON1 official sim launch no longer auto-starts `rqt_robot_steering`; use `start_steering_gui:=true` only for manual GUI tests.
[✓] FCR limiter clamp/acceleration/timeout/estop and clean-shutdown zero-burst pass at topic-output level.
[✓] FCR-side TRON1 safe mode manager added. `tron1_mode_manager_node` gates `/tron1/motion_authorized`; `tron1_safety_limiter_node` now requires both `enable_motion=true` and mode authorization before nonzero output.
[✓] 47/47 Gazebo/robot_hw_sim 安全验收用例通过。见 `docs/TRON1_SAFE_MODE_ACCEPTANCE_2026-09-04.md`。
[✓] TRON1 official controller/SDK semantics were read-only audited. See `docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md`.
[✓] TRON1 real-motion path preflight script added; it is read-only and sends no velocity commands.
[!] TRON controller watchdog clears stale velocity intent, but zero-command behavior still drifts in `WF_TRON1A + isaacgym` Gazebo. This remains a real-motion blocker.
[✓] PC reached TRON1 default IP `10.192.1.2` through `enp0s31f6` after the Ethernet link came up.
[✓] TRON1 official `pointfoot_node` connected to the real robot, loaded `WF_TRON1A` / `isaacgym`, and subscribed to `/fcr_tron/cmd_vel`.
[✓] Remote controller axes/buttons were observed through SDK `SensorJoy`; `L1 + Y/triangle` starts `WheelfootController`.
[✓] Physical motor switch/hardware action produced `Motor in damping mode`; official node then stopped the controller and exited.
[!] `L1 + X` is software `stopController()` + `abort()`, not damping/torque release.
[!] Real motion is paused. Continue with remote-controller familiarization and Gazebo/simulation first. See `docs/TRON1_REMOTE_AND_SIM_SAFETY.md`.
[✓] Sony camera UVC stream, perception smoke test, and low-speed RS2 follow verified; CRSDK node still unbuilt.
[ ] Jetson <-> TRON1 real Ethernet not verified.
[ ] TRON1 controlled low-speed real motion not accepted; first activation was too aggressive for this stage.
[ ] Full perception -> follow -> gimbal -> base chain not completed on real robot.
```

## 9. Known issues / lessons learned

### USB SSH / Mihomo route

Symptom:

```text
ssh miya@192.168.55.1 -> password denied / host key changed
ip route get 192.168.55.1 -> via 198.18.0.2 dev Mihomo
```

Cause: route/proxy path was wrong and USB gadget NIC was unreliable.

Fix: use Ethernet `ssh miya@172.31.178.242`.

Do not repeat: do not keep trying passwords on `192.168.55.1` while route points to Mihomo.

### RS2 CAN naming

Symptom: docs/scripts mention both `can0` and `can1`.

Cause: board `can0` exists, but real RS2 wiring uses external USB-CAN.

Fix: current RS2 tests use `can1`; pass launch override.

### Missing `gs_usb`

Symptom:

```text
modprobe: FATAL: Module gs_usb not found in directory /lib/modules/5.15.199-tegra
CONFIG_CAN_GS_USB is not set
```

Fix: `gs_usb.ko` was built and installed for Jetson kernel `5.15.199-tegra`; verify with `lsusb -t` and `ip link show can1`.

### Orbbec permission

Symptom: `openUsbDevice failed! status:113`.

Fix: install/reload Orbbec udev rules; verify with depth topic rate.

### OpenCV runtime split

Symptom: CMake found OpenCV but link paths such as `libopencv_core.so.4.8.0` were missing.

Fix: install matching NVIDIA `libopencv` runtime as well as `libopencv-dev`.

### Sony 相机

现象：package 存在，但没有可执行节点。

原因：CRSDK 未放入仓库；CRSDK 真 USB 流尚未验证。

后续修复：将 CRSDK 放到 `src/sony_camera_pkg/sdk/`，重新 build，验证 `lsusb` 和 topics。不要上传 SDK。

### TRON1 安全

风险：TRON1 很重，失控速度很危险。

规则：TRON1 禁止直接接裸 `/cmd_vel`。必须使用 safety limiter、物理防护、极低速度和急停。

2026-09-04 更新：

- `[VERIFIED]` FCR 侧 `tron1_safety_limiter` 的限幅、lost-command timeout、estop 路径在仿真中输出 0。
- `[VERIFIED]` `tron1_safety_limiter` 在 SIGINT/SIGTERM 关闭时发布零速度 burst；topic 尾包已确认是 0。
- `[VERIFIED]` `/safety/estop_state` 无样本或样本超时会 fail-closed，limiter 输出 0。
- `[VERIFIED]` `/safety/estop_state` 是 FCR `command_mux` 聚合的软件急停状态，不是物理 motor switch；`/tron1/limiter_clear_estop` 是同一受控 ROS_DOMAIN 内的 limiter 软件维护/恢复入口，不是硬件安全边界，真机 limiter-only 部署不应依赖它作为唯一安全门。
- `[VERIFIED]` mode manager 死亡后，limiter 会因授权信号超时归零。
- `[VERIFIED]` TRON 官方 controller 本地 watchdog 在 `/fcr_tron/cmd_vel` 输入消失后会记录 `cmd_vel timeout 0.250s exceeded; zeroing velocity command`。
- `[VERIFIED]` 官方 sim launch 默认 `start_steering_gui:=false`，安全链测试不会启动 `rqt_robot_steering`。
- `[VERIFIED]` 自动 Gazebo 验收脚本默认使用独立 `ROS_DOMAIN_ID=83`；Python 脚本是 domain 单一真源，支持 `FCR_TRON_ACCEPTANCE_ROS_DOMAIN_ID` 或 `--ros-domain-id` 覆盖，拒绝空值、`0`、前导零、非十进制或越界值，并在发布验收速度前检查 `/fcr_tron/cmd_vel` graph。
- `[VERIFIED]` `--with-gazebo` 验收用独立 60 秒上限等待 `/gazebo` 节点存在；启动后的 graph 守卫只允许 probe 和本次 Gazebo `robot_hw_node` 订阅 `/fcr_tron/cmd_vel`。
- `[BLOCKER]` `WF_TRON1A + isaacgym` Gazebo 仍有零速度命令漂移，纯 yaw 命令也会产生横移。2026-09-03 轻量 controller wheel-hold、URDF friction/contact、micro-yaw、`RL_TYPE=isaaclab` 检查都没有解决；实验性改动已撤回/恢复。应把它视作官方 sim-policy/physics blocker，不是 FCR limiter bug。在官方 SDK/controller/hardware stop 路径确认前，不运行 TRON1 真实运动。
- `[UPDATED]` 真机前 A-10 不再接受单个 `A10_CONFIRMED=yes` 作为充分证据；read-only gate 现在要求逐项确认物理停止可触达、damping 证据、`L1+X` 语义、controller watchdog 后果、Gazebo 零漂 blocker 和分步 checklist。
- `[UPDATED]` read-only gate 的进程扫描区分非预期残留和预期 live bringup；只有显式设置 `TRON1_LIVE_BRINGUP_INTENDED=yes` 时，运行中的 limiter/mode manager/官方 controller 才会进入 graph 检查路径。

## 10. Next work

Before working:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
git status --short
git remote -v
git log --oneline --decorate -5
```

Then:

1. Run the TRON1 safety acceptance checklist before any more real motion.
   - Start with [docs/TRON1_SAFETY_ACCEPTANCE_CHECKLIST.md](TRON1_SAFETY_ACCEPTANCE_CHECKLIST.md) and `tools/tron1_bringup/tron1_safety_acceptance_check.sh`.
   - Acceptance: safety gates are green or explicitly marked as blockers; do not chase full follow yet.
2. Continue TRON1 controller/SDK stop investigation before real motion.
   - Acceptance: with `/fcr_tron/cmd_vel` publisher lost or limiter killed, controller/SDK/hardware path demonstrably stops or enters a documented safe state. Gazebo-only zero-command drift is not enough for real-motion PASS; no script may claim 100% safety.
3. Implement `base_interface` / TRON1 adapter only after the interface design is reviewed.
   - Start with [docs/base_interface_tron1_adapter_design.md](base_interface_tron1_adapter_design.md).
   - Acceptance: stable command API, message/topic choices, adapter boundaries, and safety path are documented.
4. Verify Jetson <-> TRON1 Ethernet topology using official SDK and real hardware, without motor command.
   - Acceptance: `tools/tron1_bringup/pc_jetson_network_preflight.sh` reports Ethernet carrier, route not captured by Mihomo/TUN, and SSH `SSH_OK`.
5. Re-run RS2 + Orbbec coexistence after a fresh boot if hardware is connected.
   - Acceptance: `/gimbal/status connected=true`, `can1 ERROR-ACTIVE`, depth topic about 10 Hz.
6. Prepare first TRON1 real-motion checklist, but keep real motion disabled until Ethernet/SDK/no-bare-`/cmd_vel` checks pass.
   - Acceptance: controller-side timeout/watchdog, motion gate, timeout/e-stop, tiny speed, physical support/open area, and rollback commands documented.

Commit/push migration work to:

```bash
git push followrobot HEAD:main
```
