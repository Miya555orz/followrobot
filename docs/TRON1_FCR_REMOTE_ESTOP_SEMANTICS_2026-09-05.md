# TRON1 FCR Runtime Remote E-stop Semantics

Date: 2026-09-05

Scope: answer whether the TRON1 guide statement "press both sticks for global e-stop" has been verified to override FCR mode while FCR continuously sends movement commands.

Conclusion:

```text
[UNVERIFIED]
Do not treat the left/right stick press as the only reliable e-stop for the first stand, support-frame, or lawn test until it is proven on the current firmware and current FCR launch path.
```

This document is intentionally conservative. The official guide describes a global state-machine operation, but the local code path we can inspect does not prove how that event is latched, cleared, or arbitrated against `/fcr_tron/cmd_vel` while FCR is running.

## Evidence Read

| Evidence | Location | What it proves | What it does not prove |
| --- | --- | --- | --- |
| Joystick map | `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/config/joystick.yaml:3-25` | Local ROS config names A/B/X/Y/L1/R1/L2/R2, D-pad, MENU/BACK and axes. | It does not name left-stick or right-stick press buttons. |
| Official hardware ROS node | `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/src/PointfootHardwareNode.cpp:83-118`, `:187-192` | `L1 + Y` calls `startController()`. `L1 + X` calls `stopController()` then `abort()`. Joystick axes publish bare `/cmd_vel`; `subscribeSensorJoy` is registered. | It does not implement or expose the guide's "both sticks pressed" e-stop path in this ROS file. |
| Official controller timeout | `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_controllers/src/WheelfootController.cpp:41-58`, `:83-99`, `:570-608` | The command topic can be overridden by `FCR_TRON_CMD_VEL_TOPIC`; `applyCmdVelTimeout()` zeroes `commands_` and `scaled_commands_` when command input is stale. | Zero desired velocity is not physical damping, torque release, or a latched hardware e-stop. |
| FCR limiter | `src/robot_platform_pkg/src/tron1_safety_limiter_node.cpp:49-78`, `:161-230` | FCR output only becomes nonzero when `enable_motion=true`, authorization is fresh, input is fresh, and FCR estop is clear. | It only knows FCR `/safety/estop_state`; it does not subscribe to TRON1 remote e-stop feedback. |
| FCR mode manager | `src/robot_platform_pkg/src/tron1_mode_manager_node.cpp:100-115`, `:162-201` | FCR estop is latched in software and blocks `/tron1/motion_authorized`. | It does not ingest TRON1 remote e-stop state from LimX SDK. |
| Project manual | `docs/TRON1_REMOTE_CONTROLLER_MANUAL.md` | Records official guide semantics as a pending real-hardware confirmation. | It is not proof that current firmware will block the FCR command path. |

## Ten-Point Answer

| # | Question | Current answer |
| ---: | --- | --- |
| 1 | Which button/event is left/right stick press? | `[UNVERIFIED]` The official guide says "press both sticks"; local `joystick.yaml` does not name stick-press button indices. |
| 2 | What action is triggered? | `[UNVERIFIED]` The guide calls it e-stop, but local ROS code only proves `L1 + X = stopController() + abort()` and motor switch can enter damping. |
| 3 | Does it outrank continuous FCR command? | `[UNVERIFIED]` There is no inspected bridge from remote e-stop state into `tron1_safety_limiter` or `tron1_mode_manager`. |
| 4 | Can the next FCR cycle restart motion? | `[UNVERIFIED]` FCR will continue to publish if its own limiter gates remain open; whether the lower TRON1 layer ignores those commands after remote e-stop is not proven. |
| 5 | Is it latched? | `[UNVERIFIED]` The guide implies an e-stop state with a clear action, but this has not been observed through SDK state or FCR graph. |
| 6 | How is it cleared? | `[UNVERIFIED]` The guide shows a clear action, but the local code path and state feedback have not been validated on this robot. |
| 7 | Does it work during network disconnect, Jetson crash, or ROS crash? | `[UNVERIFIED]` If implemented inside the robot firmware it may, but this has not been proven. FCR software stops do not cover these failures. |
| 8 | What type of stop is it? | `[UNVERIFIED]` Treat it as an official remote/controller safety state until proven otherwise; do not call it independent hardware e-stop. |
| 9 | How are FCR and remote command sources arbitrated? | `[UNVERIFIED]` FCR uses `/fcr_tron/cmd_vel`; official joystick axes publish bare `/cmd_vel`. Current FCR tests forbid joystick motion to avoid mixed command sources. |
| 10 | Could the remote say stop while a lower layer accepts FCR command? | `[UNVERIFIED]` Yes, this remains a possible hazard until the exact firmware/controller arbitration is observed. |

## Required Verification Before Relying On It

The first proof must be non-motion or support-frame only:

1. Use `pf_sensorjoy_monitor` to identify the actual stick-press button indices.
2. Observe a robot/SDK state field, diagnostic, or log that changes when both sticks are pressed.
3. With FCR launched at `enable_motion=false`, confirm FCR remains blocked and the remote state is visible.
4. On a stand/support frame only, with physical motor switch reachable, test whether pressing both sticks prevents a later FCR nonzero command from taking effect.
5. Confirm the clear action and prove the clear does not silently reauthorize FCR motion.

Until these steps produce evidence, the only acceptable first-test safety posture is:

```text
physical motor switch / hardware stop reachable
FCR enable_motion=false at startup
A-10 gates complete
support-frame or stand first
left/right stick e-stop treated as backup only, not the sole stop
```
