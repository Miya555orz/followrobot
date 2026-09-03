# followrobot / fcr_ros2_3 -> TRON1 EDU 工程交接文档

Last updated: 2026-09-03, Asia/Shanghai

Active GitHub repo: <https://github.com/Miya555orz/followrobot>

This is the durable engineering field note for the current migration. It records code state, hardware wiring, network/SSH lessons, ROS2 launch paths, tested results, and known traps. If a fact is not verified from the current workspace or hardware session, it is explicitly marked.

## Labels

- `[VERIFIED]`: confirmed from current git/workspace, Jetson shell, or successful hardware test.
- `[UNVERIFIED]`: implemented or plausible, but not yet validated on the current real hardware.
- `[CONTEXT ONLY / NEEDS RE-VERIFY]`: came from prior conversation or older docs/logs; check again before relying on it.

## 1. Project background and final goal

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
- Put chassis-specific behavior into adapter packages, launch files, and config.
- Avoid changing business logic when swapping robot bases.
- Never let old FCR `/cmd_vel` directly control TRON1 real hardware.
- First TRON1 real tests must pass through a safety limiter with tiny speed limits, timeout stop, e-stop, and an explicit motion-enable gate.

Hardware roles:

- `[VERIFIED]` Jetson Orin Nano CLB: headless onboard computer running ROS2 Humble, RS2 driver, Orbbec driver, follow/control nodes, and future TRON1 adapter.
- `[VERIFIED]` DJI RS2: active-vision gimbal controlled by Jetson through external USB-CAN.
- `[VERIFIED]` Orbbec Gemini 335: depth camera, currently verified as depth-only 424x240@10Hz.
- `[VERIFIED]` Sony ZV-E10M2: UVC mode enumerates as `/dev/video8`, publishes `/sony/image_raw`, and has passed YOLO/person detection, tracking, aim-target smoke tests, and conservative-medium RS2 closed-loop follow; proprietary CRSDK node is still not staged.
- `[VERIFIED]` Final 2026-09-02 Sony->RS2 lab profile used `gimbal_visual_servo_low_speed_lab.yaml` with yaw `0.12 rad/s` and pitch `0.075 rad/s` limits after direction was confirmed normal. All related lab processes were stopped at end of day.
- `[VERIFIED]` Lightweight live viewer: `tools/visualization/ros_image_mjpeg_viewer.py` serves Sony raw and OpenCV debug images at `http://<JETSON_IP>:8088/`.
- `[PARTIAL]` TRON1 EDU: target robot base. PC Ethernet, SDK connection, remote-controller input, and controller activation were observed on 2026-09-03, but real motion is paused because activation felt too strong/fast for this stage. Continue in simulation first.

## 2. Current code and directories

Active ROS workspace:

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
| `/safety/estop_state` | `std_msgs/Bool` | e-stop gate |

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

### Sony camera

Symptom: package exists but no executable.

Cause: CRSDK not staged; real USB stream not verified.

Fix later: stage CRSDK under `src/sony_camera_pkg/sdk/`, rebuild, verify `lsusb`/topics. Do not upload SDK.

### TRON1 safety

Risk: TRON1 is heavy; uncontrolled speed is dangerous.

Rule: no direct `/cmd_vel` to TRON1. Use safety limiter, physical protection, tiny speeds, and e-stop.

2026-09-03 update:

- `[VERIFIED]` FCR-side `tron1_safety_limiter` clamp / lost-command timeout / estop paths output zero in simulation.
- `[VERIFIED]` `tron1_safety_limiter` publishes a zero burst during SIGINT/SIGTERM shutdown; topic tail was confirmed zero.
- `[VERIFIED]` TRON official controller local watchdog logs `cmd_vel timeout 0.250s exceeded; zeroing velocity command` after `/fcr_tron/cmd_vel` input disappears.
- `[VERIFIED]` Official sim launch now has `start_steering_gui:=false` by default, so `rqt_robot_steering` is not started during safety-chain tests.
- `[BLOCKER]` `WF_TRON1A + isaacgym` Gazebo still drifts at zero velocity command, and pure yaw commands produce lateral translation. On 2026-09-03, lightweight controller wheel-hold, URDF friction/contact, micro-yaw, and `RL_TYPE=isaaclab` checks did not solve it; the experimental changes were withdrawn/restored. Treat this as an official sim-policy/physics blocker, not an FCR limiter bug. Do not run real TRON1 motion until the official SDK/controller/hardware stop path is verified.

## 10. Next work

Before working:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
git status --short
git remote -v
git log --oneline --decorate -5
```

Then:

1. Re-run RS2 + Orbbec coexistence after a fresh boot.
   - Acceptance: `/gimbal/status connected=true`, `can1 ERROR-ACTIVE`, depth topic about 10 Hz.
2. Design `base_interface` before large refactor.
   - Acceptance: stable command API, message/topic choices, adapter boundaries, and safety path are documented.
3. Continue TRON1 controller/SDK stop investigation before real motion.
   - Acceptance: with `/fcr_tron/cmd_vel` publisher lost or limiter killed, controller/SDK/hardware path demonstrably stops or enters a documented safe state. Gazebo-only zero-command drift is not enough for real-motion PASS.
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
