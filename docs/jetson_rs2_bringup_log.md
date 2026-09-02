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

- Today’s hardware target is Jetson Orin Nano CLB + board CAN `can0` + DJI RS2
  + Orbbec/Gemini depth camera.
- `can0` is the correct real interface for today; it is Jetson `mttcan`, not an
  external `gs_usb` USB-CAN adapter.
- Sony camera is unavailable today and must not be treated as a blocker.
- TRON1 real chassis motion remains disabled/out of scope.
- Codex cannot currently run Jetson commands over SSH because passwordless SSH
  is not configured. The old SSH host key for `192.168.55.1` was removed after
  the Jetson reflash and the new key was accepted, but authentication still
  requires the user’s password or key setup.
