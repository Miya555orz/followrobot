# Jetson + DJI RS2 + depth camera test plan — 2026-09-02

This is the morning runbook for the current TRON1 migration stage. The goal is
to bring up the Jetson-local communication chain only:

```text
Jetson Orin Nano
  + on-board SocketCAN can0
  + DJI RS2 gimbal
  + Orbbec/Gemini depth camera
```

TRON1 real chassis motion is out of scope for this test day. Do not start the
official TRON1 hardware controller and do not publish chassis commands to a real
robot.

## Current known Jetson state

- JetPack / Jetson Linux: R36.5.2, Ubuntu 22.04.5, kernel `5.15.199-tegra`.
- Root filesystem: NVMe `/dev/nvme0n1p1`.
- ROS 2: Humble available at `/opt/ros/humble/bin/ros2`.
- Workspace: `~/follow_ws`.
- Built/visible FCR packages on Jetson:
  - `vision_servo_msgs`
  - `robot_platform_pkg`
  - `teleop_control_pkg`
- `bringup_pkg` is not required for the minimum RS2 test. It failed on Jetson
  because its launch-only package declares runtime dependencies on packages that
  were not built yet, including Sony/perception/simulation/voice packages.
- Jetson on-board CAN exists as `can0`, driver `mttcan`, currently `DOWN /
  STOPPED` until configured.
- Sony camera is unavailable for this test. Keep all Sony-dependent launch
  arguments disabled.

## Minimal code paths

### RS2 gimbal

```text
/cmd_gimbal
  -> robot_platform_pkg/gimbal_driver_node
  -> SocketCAN can0
  -> DJI RS2
  -> /gimbal/status
```

The RS2 protocol uses standard CAN IDs:

- host → gimbal: `0x223`
- gimbal → host: `0x222`

The safest first control mode is:

```text
control_mode:=incremental_position
```

In this project that means the ROS side still accepts `GimbalCmd` rate fields,
but the driver integrates them into small bounded position steps and sends the
RS2 absolute position command that has been most reliable in prior tests.

### Depth camera

Start with the Orbbec/Gemini driver only:

```text
orbbec_camera launch
  -> /camera/depth/image_raw
  -> /camera/depth/camera_info
```

Do not start the Sony-based detector/tracker chain tomorrow unless a separate
RGB input is prepared. Without Sony, `depth_fusion_node` cannot complete the
production `/perception/tracks + Gemini depth -> /perception/targets_3d` chain,
because current fusion parameters expect:

```text
/perception/tracks
/camera/depth/image_raw
/camera/depth/camera_info
/sony/camera_info
```

For tomorrow, depth-camera success means: the camera is visible on USB, the
Orbbec node starts, and depth image plus depth CameraInfo publish at a stable
rate.

### TRON1 safety preparation

The prepared safety path is:

```text
/fcr/cmd_vel_stamped
  -> tron1_safety_limiter_node
  -> /fcr_tron/cmd_vel
  -> future TRON1 official robot_hw_node
```

Current safety defaults are deliberately tiny:

- `enable_motion=false`
- `max_linear_x=0.03 m/s`
- `max_angular_z=0.10 rad/s`
- lateral motion disabled for first real tests
- input timeout `0.25 s`

TRON1 must never subscribe to a naked `/cmd_vel` in the real test. It should
consume only `/fcr_tron/cmd_vel` after the safety limiter.

## Morning zero-to-test commands

Run these on the Jetson terminal. The prompt should look like:

```bash
miya@ubuntu:~$
```

### T0 — environment check

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

hostname
cat /etc/nv_tegra_release
which ros2
ros2 topic list
ros2 pkg list | grep -E 'vision_servo_msgs|robot_platform_pkg|teleop_control_pkg'
ip -details link show can0
```

Expected:

- `which ros2` prints `/opt/ros/humble/bin/ros2`.
- The three FCR packages are listed.
- `can0` exists. It may still be `DOWN` before T1.

### T1 — bring up Jetson on-board can0

```bash
cd ~/follow_ws/src/fcr_ros2_3
bash tools/tron1_bringup/setup_jetson_mttcan_can0.sh
```

Expected:

```text
can state ERROR-ACTIVE
bitrate 1000000
JETSON_CAN_INTERFACE=can0
```

If it says `NO-CARRIER`, `BUS-OFF`, or errors increase quickly, stop and check
RS2 CAN wiring, transceiver power, GND, CAN_H/CAN_L, and termination.

### T2 — RS2 CAN bare preflight

```bash
cd ~/follow_ws/src/fcr_ros2_3
bash tools/tron1_bringup/rs2_can_preflight.sh can0
```

This script listens for `0x222` for a few seconds. No frames at this stage is
not a final failure, because RS2 may stay silent until the ROS driver sends a
query. The important check is that can0 remains `UP / ERROR-ACTIVE` and CAN
error counters do not explode.

### T3 — start RS2 ROS 2 driver

Terminal 1:

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch robot_platform_pkg gimbal_bringup.launch.py \
  use_sim:=false \
  can_interface:=can0 \
  control_mode:=incremental_position
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 lifecycle get /gimbal_driver
ros2 param get /gimbal_driver can_interface
ros2 param get /gimbal_driver control_mode
ros2 topic echo /gimbal/status --once
```

Expected:

- lifecycle is `active`
- `can_interface` is `can0`
- `control_mode` is `incremental_position`
- `/gimbal/status.connected` eventually becomes `true`
- `crc_error_count`, `can_error_count`, `parse_error_count` do not keep rising

### T4 — depth camera driver and topic check

First check whether the Orbbec packages are available:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 pkg list | grep -E 'orbbec_camera$|orbbec_camera_msgs'
```

If they are missing, build them with low parallelism:

```bash
cd ~/follow_ws
source /opt/ros/humble/setup.bash

MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
  --packages-up-to orbbec_camera

source ~/follow_ws/install/setup.bash
```

If the build asks for missing apt packages, install only the missing packages it
prints, then repeat the build. Do not clean `build/`, `install/`, or `log/`
unless CMake cache corruption is confirmed.

Then start the camera in a new terminal. Use the low-CPU launch first:

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

In another terminal:

```bash
cd ~/follow_ws/src/fcr_ros2_3
bash tools/tron1_bringup/depth_camera_preflight.sh
```

Expected:

- `/camera/depth/image_raw` exists.
- `/camera/depth/camera_info` exists.
- `ros2 topic hz /camera/depth/image_raw` is close to 10 Hz.

If this launch does not match the actual depth camera model, use `ros2 launch
orbbec_camera <TAB><TAB>` or choose the model-specific launch in
`src/orbbec_camera/launch/`.

### T5 — manual RS2 small motion

Only do this after T3 shows the RS2 driver is active and the gimbal is safely
fixed on the bench.

Small debug position service:

```bash
ros2 service call /gimbal/debug_position std_srvs/srv/Trigger {}
```

Then immediately check:

```bash
ros2 topic echo /gimbal/status --once
```

If service motion is too large or direction is uncertain, stop here and use
smaller keyboard nudge parameters instead of repeated direct topic commands.

Keyboard nudge path, optional:

Terminal 1 already runs `gimbal_driver_node`. Terminal 2 starts the mux:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 launch teleop_control_pkg remote_control.launch.py \
  start_keyboard:=false \
  start_manual_jog:=false
```

Terminal 3 must be interactive:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 run teleop_control_pkg keyboard_platform_teleop \
  --ros-args \
  -p gimbal_yaw_step_deg:=0.5 \
  -p gimbal_pitch_step_deg:=0.5 \
  -p gimbal_nudge_duration_sec:=0.1
```

Use only arrow keys for the gimbal. Do not use `W/A/S/D/Q/E`, because those are
reserved for chassis commands in the combined keyboard tool.

### T6 — depth camera + gimbal coexistence

Run at the same time:

- Terminal 1: RS2 `gimbal_bringup.launch.py`
- Terminal 2: Orbbec depth-camera launch
- Terminal 3: graph snapshot

```bash
cd ~/follow_ws/src/fcr_ros2_3
bash tools/tron1_bringup/ros2_comm_snapshot.sh
```

Pass criteria:

- `/gimbal/status` publishes.
- `/camera/depth/image_raw` publishes.
- `/camera/depth/camera_info` publishes.
- No chassis hardware controller is running.
- No real TRON1 motion topic is being consumed.

### T7 — before TRON1 simulation or later real chassis test

Do not start real TRON1 hardware tomorrow. Preparation-only check:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 run robot_platform_pkg tron1_safety_limiter_node --ros-args \
  -p enable_motion:=false \
  -p enable_lateral:=false \
  -p max_linear_x:=0.03 \
  -p max_angular_z:=0.10 \
  -p input_timeout_sec:=0.25
```

In another terminal, prove the gate is closed:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash

ros2 topic pub --once /fcr/cmd_vel_stamped geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 1.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0}}}"

ros2 topic echo /fcr_tron/cmd_vel --once
```

Expected `/fcr_tron/cmd_vel` is all zero because `enable_motion=false`.

## Risk points

1. CAN physical layer is the most likely blocker: RS2 needs correct CAN_H,
   CAN_L, shared GND, transceiver power, and termination.
2. Jetson `can0` is `mttcan`; do not follow old USB-CAN-only docs that require
   `driver=gs_usb` unless an external USB-CAN adapter is physically used.
3. `candump` may show no RS2 traffic until the ROS gimbal driver sends query
   frames. Judge by driver status and counters, not by bare listen alone.
4. Sony is unavailable, so production perception tracking/fusion is not a
   success criterion tomorrow.
5. Do not start TRON1 real motion. The heavy chassis must later be tested with
   `enable_motion=false` first, then tiny limits, physical protection, and a
   human watching power/estop.
