# TRON1 Phase Freeze

Date: 2026-09-05

Purpose: freeze the current non-motion TRON1 stage before any future support-frame or lawn test.

## Scope

This freeze covers:

- PC to Jetson USB SSH recovery and routing notes.
- Jetson to TRON1 Ethernet network-only route/ping PASS.
- FCR TRON1 limiter/mode-manager safety semantics.
- Gazebo safety semantics and known zero-drift blocker.
- Future real-test runbook.
- FCR runtime remote e-stop evidence gap.

It does not authorize ground movement.

## Freeze Identity

```text
Branch: main
Active push remote: followrobot
Rollback point before this freeze: 9c63c11 Make Jetson TRON1 route setup repeatable
Freeze commit: the followrobot/main commit containing this document; record exact hash from `git log -1 --oneline` after push.
Workspace policy: do not push to origin/cuiangA unless explicitly requested.
```

This freeze is only effective after the document set is committed and pushed to `followrobot/main`; before that, use the rollback point above as the last durable state.

## Tested Configuration

```text
ROS 2: Humble
FCR workspace: /home/miya/follow_ws
FCR repo: /home/miya/follow_ws/src/fcr_ros2_3
TRON1 official workspace: /home/miya/limx_ws
TRON robot type: WF_TRON1A
RL type: isaacgym
FCR TRON output topic: /fcr_tron/cmd_vel
Forbidden real path: bare /cmd_vel as FCR output
Default launch gates: start_tron_hw=false, enable_motion=false, allow_tron_follow_motion=false
```

## Evidence Included In This Freeze

| Evidence | File |
| --- | --- |
| PC direct TRON1 read-only preflight | `docs/ai/TRON1_PC_DIRECT_PREFLIGHT_2026-09-05.md` |
| PC USB SSH to Jetson quickstart | `docs/ai/JETSON_USB_SSH_QUICKSTART_2026-09-05.md` |
| Jetson to TRON1 network-only PASS | `docs/ai/TRON1_JETSON_NETWORK_PREFLIGHT_2026-09-05.md` |
| Repeatable Jetson route setup | `docs/ai/TRON1_JETSON_NETWORK_SETUP_REPEATABLE_2026-09-05.md` |
| Gazebo safety semantics freeze | `docs/TRON1_GAZEBO_SAFETY_SEMANTICS_FREEZE_2026-09-05.md` |
| FCR remote e-stop semantics | `docs/TRON1_FCR_REMOTE_ESTOP_SEMANTICS_2026-09-05.md` |
| Future real-test runbook | `docs/TRON1_REAL_TEST_RUNBOOK.md` |

## Known Limitations

- Gazebo `WF_TRON1A + isaacgym` zero-command drift remains a blocker for claiming physical stop distance.
- Official controller watchdog zeros desired velocity, not physical damping.
- `L1 + X` is `stopController() + abort()`, not confirmed damping.
- Left/right stick "global e-stop" from the guide is not yet verified to override FCR continuous commands.
- FCR software estop and limiter clear topics are ROS graph semantics, not independent hardware stop chains.
- Support-frame/stand low-speed motion has not been completed in this freeze.
- Lawn testing has not started.

## Local Validation During Freeze

```text
git diff --check: PASS
bash -n tools/tron1_bringup/*.sh: PASS
python3 -m py_compile tools/tron1_bringup/tron1_safe_mode_acceptance.py src/teleop_control_pkg/scripts/fcr_mode_console.py: PASS
colcon build --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg: PASS
colcon test --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg: PASS, 4 test targets / 18 test cases
tron1_safety_acceptance_check.sh: FAIL=0 WARN=0 BLOCK=8 while tree was dirty and A-10/live graph were intentionally incomplete
```

The read-only gate `BLOCK` result above is the expected safety state for this freeze. It confirms no final real-motion PASS is being claimed.

## Hardware Configuration Snapshot

```text
Jetson Orin Nano CLB: flashed and usable, JetPack 6.2.3 / Jetson Linux R36.5.2
Sony UVC camera: verified with perception/tracking/aim target
DJI RS2: verified through external USB-CAN can1/gs_usb
Orbbec Gemini 335: depth stream verified on Jetson USB3
TRON1 Ethernet IP: 10.192.1.2
Jetson wired TRON1 interface observed: enP8p1s0
Jetson TRON1 route observed: 10.192.1.2 dev enP8p1s0 src 10.192.1.200
PC Jetson USB IP: 192.168.55.1, with host-side USB NIC route required to avoid Mihomo/TUN
```

## Phase Result

```text
TRON1 real motion: PAUSED
Network-only Jetson->TRON1: PASS
PC->Jetson USB SSH: documented and route pitfall understood
Gazebo safety: command-layer PASS, pose-drift blocker retained
Runbook: prepared for future support-frame and lawn work
Next allowed step: support-frame/stand preparation and read-only gates, not ground movement
```
