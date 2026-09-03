# TRON1 Safety Acceptance Checklist

Purpose: move TRON1 work from “it can move” back to “it is controllable, stoppable, and safety evidence exists.”

Scope: TRON1 EDU / `WF_TRON1A`, PC or Jetson ROS 2 Humble, official LimX controller, FCR safety limiter.

Current rule: do not run real TRON1 motion until this checklist is green. Gazebo pose drift is a known official sim-policy/physics blocker, so simulation is used for topic safety and controller wiring, not for proving real stopping distance.

## Safety architecture

```text
PC / Jetson
  -> FCR command source
  -> /fcr/cmd_vel_stamped              geometry_msgs/TwistStamped
  -> tron1_safety_limiter
  -> /fcr_tron/cmd_vel                 geometry_msgs/Twist
  -> TRON1 official controller
  -> TRON1
```

Hard rule:

- TRON1 official controller must subscribe to `/fcr_tron/cmd_vel`.
- Nothing should drive TRON1 through bare `/cmd_vel`.
- `tron1_safety_limiter` is the only allowed publisher of `/fcr_tron/cmd_vel`.
- Real-robot default is `enable_motion=false`.

## Stage A: safety baseline, no motion goal

Run:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./tools/tron1_bringup/tron1_safety_acceptance_check.sh
```

Acceptance:

| ID | Check | PASS |
| --- | --- | --- |
| A-01 | No stale `gazebo`, `pointfoot_node`, `tron1_safety_limiter`, or raw `ros2 topic pub` process before testing | no unexpected process |
| A-02 | `/fcr_tron/cmd_vel` has exactly one publisher | publisher is `tron1_safety_limiter` |
| A-03 | TRON controller subscribes `/fcr_tron/cmd_vel` | subscriber is `robot_hw_node` / official controller |
| A-04 | No bare `/cmd_vel` path is used for TRON1 | official controller is launched with `fcr_cmd_vel_topic:=/fcr_tron/cmd_vel` |
| A-05 | `enable_motion=false` forces zero output | nonzero upstream command still gives zero `/fcr_tron/cmd_vel` |
| A-06 | `enable_motion=true` clamps output | max `linear.x <= 0.03`, `linear.y == 0`, max `angular.z <= 0.10` |
| A-07 | lost command timeout | output returns zero after input stops |
| A-08 | `/safety/estop_state=true` | output immediately becomes zero |
| A-09 | limiter shutdown | SIGINT/SIGTERM publishes zero burst |
| A-10 | limiter or publisher crash | official controller watchdog/hardware stop behavior is understood before real motion |

Notes:

- A-01 to A-09 can be validated in simulation/topic-level tests.
- A-10 is the real blocker before true low-speed robot motion.
- `L1 + X` is software stop/abort, not proven damping.
- Physical motor switch / hardware action was observed to produce `Motor in damping mode`.

## Stage B: official controller and remote state machine

Read-only questions to answer before more real motion:

- What state does `WheelfootController` enter after start?
- Is zero `/cmd_vel` dynamic balancing, braking, damping, or only zero velocity intent?
- Which remote key combinations start/stop controller?
- Is there an official damping / lock / sit / zero-torque API?
- What happens if `/fcr_tron/cmd_vel` publisher disappears?
- What happens if the SDK process dies?

Evidence files:

- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/src/PointfootHardwareNode.cpp`
- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_controllers/src/WheelfootController.cpp`
- [docs/TRON1_REMOTE_AND_SIM_SAFETY.md](TRON1_REMOTE_AND_SIM_SAFETY.md)
- [docs/tron1_external_repo_note.md](tron1_external_repo_note.md)

## Stage C: interface isolation

Before reconnecting follow logic, all upper-level code should see only a unified base command interface. TRON1 details stay below the adapter/limiter boundary.

Required:

- follow/tracking code publishes one generic base command topic.
- TRON1 adapter converts generic command to `/fcr/cmd_vel_stamped` or directly to limiter input.
- `tron1_safety_limiter` remains the hard gate.
- launch/config selects `omni` or `tron1`; perception and tracking do not contain TRON1-specific logic.

Design doc:

- [docs/base_interface_tron1_adapter_design.md](base_interface_tron1_adapter_design.md)

## Stage D: real robot, lowest risk only

Only after A/B/C are green:

1. Real robot supported or wheels off ground.
2. Confirm hardware damping / emergency action is reachable.
3. Start with `enable_motion=false`.
4. Confirm `/fcr_tron/cmd_vel` remains zero under nonzero upstream input.
5. Enable motion only for a short pulse:
   - `linear.x = 0.01~0.02 m/s`
   - duration `0.3~0.5 s`
6. Stop input and verify stop.
7. Test software estop.
8. Test physical stop/damping.
9. Only then test tiny yaw:
   - `angular.z = 0.03~0.05 rad/s`

Do not start with Sony + RS2 + TRON full follow.

## Current conclusion

```text
RS2 + Sony visual following is already working.
TRON1 is not yet a follow base; it is a safety-controlled base interface under bringup.
```
