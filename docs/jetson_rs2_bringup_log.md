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
TODO paste output here
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

- Jetson ROS 2 status: TODO
- FCR workspace status: TODO
- CAN adapter status: TODO
- RS2 communication status: TODO
- Blockers: TODO
