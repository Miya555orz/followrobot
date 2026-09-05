# TRON1 PC Direct Wired Preflight Record - 2026-09-05

Scope: read-only PC-to-TRON1 network and software-default check. No ROS node was launched, no controller was activated, and no velocity command was published.

## Physical Setup

- PC wired interface: `enp0s31f6`
- TRON1 default IP: `10.192.1.2`
- Temporary PC-side direct route used during the check:
  - `10.192.1.2 dev enp0s31f6 src 10.192.1.200`
- Test location: small office.
- Safety decision after the check: disconnect PC from TRON1 and keep real motion paused.

## Observed Result

```text
ip route get 10.192.1.2
10.192.1.2 dev enp0s31f6 src 10.192.1.200 uid 1000
```

```text
ping -c 1 -W 1 10.192.1.2
1 packets transmitted, 1 received, 0% packet loss
```

```text
TRON_LINK_IFACE=enp0s31f6 ./tools/tron1_bringup/tron1_real_motion_path_preflight.sh
[PASS] TRON_IP route uses direct wired-looking dev enp0s31f6.
[PASS] TRON_IP responds to one read-only ping.
[PASS] FCR/TRON launch default keeps start_tron_hw=false.
[PASS] FCR/TRON launch default keeps enable_motion=false.
[INFO] Official robot_hw launch exposes /cmd_vel as a default; real bringup must use the FCR launch override to /fcr_tron/cmd_vel.
Summary: PASS=4 WARN=0 BLOCK=0 FAIL=0
```

```text
ros2 topic list | sort
/parameter_events
/rosout

ros2 topic info -v /fcr_tron/cmd_vel
Unknown topic '/fcr_tron/cmd_vel'

ros2 topic info -v /cmd_vel
Unknown topic '/cmd_vel'
```

## Interpretation

This proves the PC-side direct route and read-only preflight can pass when the cable and route are correct. It is not a real-motion permit and not a 100% safety guarantee. Real movement remains blocked until the A-10 items, Gazebo zero-drift blocker, controller/SDK/hardware-stop review, and the staged checklist are completed in a suitable protected space.

## Next Non-Motion Step

Prepare this network-only topology:

```text
PC --USB/SSH--> Jetson
Jetson --Ethernet--> TRON1
```

Run only Jetson-side route and ping checks plus `tools/tron1_bringup/jetson_tron1_network_preflight.sh`. Do not launch `robot_hw`, do not activate the controller, and do not publish any velocity topic.
