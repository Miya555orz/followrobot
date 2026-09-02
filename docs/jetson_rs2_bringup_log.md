# Jetson Orin Nano + RS2 CAN Bring-up Log

Date: 2026-09-01

Goal: verify whether the Jetson Orin Nano can run the ROS 2/FCR base environment and whether DJI RS2 CAN hardware is visible to Linux.

## Step J0: identify the machine

Expected:

- The terminal is on Jetson, not the laptop.
- Ubuntu 22.04 / ROS 2 Humble is available, or missing items are clearly recorded.

Commands:

```bash
hostname
uname -a
lsb_release -a
which ros2
ros2 --version
```

Result:

```text
hostname: ubuntu
Ubuntu: 22.04.5 LTS
Kernel: 5.15.199-tegra aarch64
L4T: R36 (release), REVISION: 5.2, GCID: 46426093, BOARD: generic
Root filesystem: /dev/nvme0n1p1, 233G total, 214G available
Network:
  wlP1p1s0 DOWN
  can0 DOWN
  enP8p1s0 UP 172.31.178.242/24
  l4tbr0 UP 192.168.55.1/24
Storage:
  nvme0n1 238.5G
  nvme0n1p1 mounted at /
```

## Step J1: run read-only preflight

Command:

```bash
cd ~/follow_ws/src/fcr_ros2_3
bash tools/tron1_bringup/jetson_rs2_preflight.sh
```

Result:

```text
TODO paste output here
```

## Step C0: CAN device detection

Expected:

- SocketCAN device such as `can0`, or
- USB serial device such as `/dev/ttyUSB0` / `/dev/ttyACM0`, depending on the USB-CAN adapter.

Commands:

```bash
ip -details link show
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg --ctime | tail -120
```

Result:

```text
TODO paste output here
```

## Step C1: CAN interface setup

Only run this after C0 confirms the device is `can0`.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

Result:

```text
TODO paste output here
```

## Step C2: CAN traffic sniff

Only run this after C1 succeeds.

```bash
candump can0
```

Result:

```text
TODO paste output here
```

## Step R0: RS2 ROS node

Only run this after CAN interface exists.

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg gimbal_bringup.launch.py
```

Result:

```text
TODO paste output here
```

## Conclusion

- Jetson flash status: PASS; JetPack 6.2.3 / L4T R36.5.2 booted successfully from NVMe.
- Jetson network status: PASS; USB gadget SSH works at 192.168.55.1, RJ45 received 172.31.178.242/24.
- Jetson ROS 2 status: PASS; `/opt/ros/humble/bin/ros2` is available and `ros2 topic list` returns `/parameter_events` and `/rosout`.
- FCR Jetson core package status: PASS for `vision_servo_msgs`, `robot_platform_pkg`, and `teleop_control_pkg`. `bringup_pkg` is optional for the minimum RS2 test and failed only because broad runtime packages were not yet built.
- CAN adapter status: pending; Jetson built-in `can0` is present as `mttcan` but DOWN/STOPPED. Configure at 1 Mbit/s only after confirming the RS2 CAN wiring/transceiver path.
- RS2 communication status: pending; test after CAN interface appears.
- Blockers: none for OS boot or ROS 2 core. Next risk area is CAN/RS2 wiring and depth-camera package/udev setup on Jetson.

## 2026-09-02 session notes

- Earlier planning assumed Jetson Orin Nano CLB board CAN `can0` + DJI RS2 +
  Orbbec/Gemini depth camera.
- Actual 2026-09-02 RS2 setup switched to an external USB-to-CAN adapter because
  the CLB board CAN path is exposed through the gold-finger connector. In this
  setup, Jetson `can0` remains the unused board `mttcan` interface and the RS2
  USB-CAN adapter is `can1` with the `gs_usb` driver.
- Sony camera is unavailable today and must not be treated as a blocker.
- TRON1 real chassis motion remains disabled/out of scope.
- Codex cannot currently run Jetson commands over SSH because passwordless SSH
  is not configured. The old SSH host key for `192.168.55.1` was removed after
  the Jetson reflash and the new key was accepted, but authentication still
  requires the user’s password or key setup.

## 2026-09-02 USB-CAN recovery and RS2 validation

Updated physical setup:

- RS2 is connected through an external USB-to-CAN adapter plugged into the
  Jetson USB port.
- The Jetson CLB board CAN remains `can0`, but it is not used for this setup
  because the board CAN path is exposed through the CLB gold-finger connector
  and is not convenient as a simple 2-pin CAN connection.
- The active RS2 CAN interface is therefore the USB-CAN adapter, which appears
  as `can1` after loading `gs_usb`.

USB-CAN device observed on Jetson:

```text
Bus 001 Device 004: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
```

Jetson kernel status before recovery:

```text
5.15.199-tegra
# CONFIG_CAN_GS_USB is not set
# CONFIG_CAN_SLCAN is not set
CONFIG_CAN_DEV=m
```

Because Jetson Linux R36.5.2 did not ship a `gs_usb` module in this image, the
USB-CAN adapter was visible in `lsusb` but did not create a `canX` interface.
The workaround used in this session was:

1. Download `drivers/net/can/usb/gs_usb.c` from Linux stable `v5.15.199`.
2. Build it out-of-tree against `/lib/modules/5.15.199-tegra/build`.
3. Load it with `insmod`.
4. Install it into:

```text
/lib/modules/5.15.199-tegra/kernel/drivers/net/can/usb/gs_usb.ko
```

5. Run `depmod -a`, after which `modinfo gs_usb` resolves the module.

Successful module and CAN state:

```text
gs_usb                 24576  0
can_dev                36864  2 mttcan,gs_usb

8: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
    can state ERROR-ACTIVE restart-ms 100
    bitrate 1000000
    gs_usb ... parentbus usb parentdev 1-2.3:1.0
    RX/TX errors: 0
```

RS2 ROS2 driver launch:

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg gimbal_bringup.launch.py \
  use_sim:=false \
  can_interface:=can1 \
  control_mode:=incremental_position
```

Driver result:

```text
[DJIRS2Gimbal] 已连接 'can1' (发→0x223, 收←0x222)
云台驱动配置完成 (sim=0, can=can1, mode=incremental_position)
云台驱动已激活
```

ROS2 status:

```text
ros2 lifecycle get /gimbal_driver
active [3]

ros2 param get /gimbal_driver can_interface
String value is: can1

ros2 topic echo /gimbal/status --once
connected: true
crc_error_count: 0
can_error_count: 0
parse_error_count: 0
```

Small gimbal motion test:

```bash
timeout 2s ros2 topic pub -r 10 /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
  "{yaw_rate: -0.05, pitch_rate: 0.0, hold_yaw: false, hold_pitch: true}"

ros2 topic pub --once /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
  "{yaw_rate: 0.0, pitch_rate: 0.0, hold_yaw: true, hold_pitch: true}"
```

Observed yaw changed from approximately `2.9967 rad` to `2.9671 rad`, while
`connected=true` and all CAN/CRC/parse error counters remained zero.

Current result:

- Jetson → USB-CAN (`can1`) → DJI RS2 CAN communication: PASS.
- `robot_platform_pkg/gimbal_driver_node` lifecycle activation: PASS.
- `/gimbal/status` feedback: PASS.
- Minimal RS2 yaw command: PASS.
- TRON1 real chassis motion: not started and remains disabled.
