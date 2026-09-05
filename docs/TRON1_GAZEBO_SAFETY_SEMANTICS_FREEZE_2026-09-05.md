# TRON1 Gazebo Safety Semantics Freeze

Date: 2026-09-05

Scope: close the current Gazebo safety discussion without hiding known behavior. This is a simulation and software-safety freeze, not a real-motion permit.

## Bottom Line

Gazebo blockers are not "all solved" in the sense of proving a physical stop distance. The FCR software gate behavior is verified at the topic/output layer, while the official `WF_TRON1A + isaacgym` Gazebo robot can still drift or slide under zero desired velocity. That remaining behavior is carried as an official sim-policy/physics blocker and must not be masked by large damping/friction tweaks.

Current interpretation:

```text
FCR limiter output safety: PASS at topic layer.
Official controller stale-command watchdog: PASS for zeroing desired velocity.
Gazebo pose hold / zero-drift: NOT A PASS; known blocker.
Physical damping / hardware stop: observed separately on real hardware, not proven by Gazebo.
Real ground motion: still paused.
```

## Scenario Matrix

PASS in this matrix means the command-intent layer, guard behavior, or blocker-handling semantics are understood. It never means physical stop distance, hardware damping, or ground-motion permission.

| Scenario | Root cause / semantics | Location | Minimal repro | Validation status | PASS/FAIL |
| --- | --- | --- | --- | --- | --- |
| A. `/cmd_vel=0` drift | Zero desired velocity is not a hard physical stop for the RL balancing policy. Gazebo may continue residual motion from policy, contact, inertia, friction, or initial pose. | `/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_controllers/src/WheelfootController.cpp`; `docs/tron1_migration_gate_report_2026-09-03.md` | Start official sim with `WF_TRON1A + isaacgym`, publish or maintain zero desired velocity, observe model pose. | Semantics documented; no masking parameter change accepted. | FAIL as "pose hold"; PASS as "known blocker correctly carried". |
| B. Command publisher disappears | FCR limiter treats stale input as stop, and official controller watchdog also zeroes stale desired velocity. | `src/robot_platform_pkg/src/tron1_safety_limiter_node.cpp`; official `WheelfootController.cpp` | Stop publisher feeding `/fcr/cmd_vel_stamped`; observe `/fcr_tron/cmd_vel` zero and controller timeout log in sim. | Covered by prior 47/47 acceptance plus controller code read. | PASS at command-intent layer. |
| C. Controller node crash | If FCR mode manager dies, auth freshness times out and limiter zeros output. If limiter itself is killed without clean shutdown, the official controller watchdog is the remaining software fallback. | `src/robot_platform_pkg/src/tron1_mode_manager_node.cpp`; `src/robot_platform_pkg/src/tron1_safety_limiter_node.cpp`; official controller timeout | Kill mode manager in sim; separately reason about limiter crash. | Mode-manager death covered. Limiter hard crash still depends on official watchdog and physical stop. | PASS for mode-manager death; BLOCK for relying on limiter hard-crash alone. |
| D. Timeout | FCR input timeout and official controller cmd timeout both zero desired command. | Same as B. | Stop input for longer than configured timeout. | Prior acceptance validates limiter; official code validates command watchdog. | PASS at desired-command layer. |
| E. ROS2 graph / communication anomaly | Missing graph, stale auth, stale estop sample, unexpected publisher/subscriber, Gazebo/steering leftovers, or bare `/cmd_vel` are fail-closed gates. | `tools/tron1_bringup/tron1_safety_acceptance_check.sh`; `tools/tron1_bringup/tron1_real_motion_path_preflight.sh` | Run read-only gates with and without expected live graph. | Gates block by default and require explicit live-bringup intent. | PASS for fail-closed behavior. |
| F. Gazebo natural sliding / inertia / friction | Gazebo pose drift is not by itself proof of a FCR command leak. It may be physics/contact/RL residual. | `docs/base_interface_tron1_adapter_design.md`; this file | Observe pose under zero desired velocity while `/fcr_tron/cmd_vel` is zero. | Treated as known limitation; do not tune it away without controller-level proof. | BLOCK for using Gazebo pose as real-stop evidence. |
| G. Controller still outputs nonzero control | The RL controller may keep producing joint/wheel control to balance while desired velocity is zero. That is expected for a balancing robot and not equivalent to commanded locomotion permission. | Official `WheelfootController.cpp` WALK policy update loop | Inspect controller update loop and logs; compare command intent to model pose. | Semantics documented; hardware consequence still requires stand/support-frame testing. | PASS as documented behavior; BLOCK as real-motion guarantee. |
| Stop command | FCR software stop and estop force `/fcr_tron/cmd_vel` to zero. `L1 + X` stops the official controller process but is not damping. | FCR limiter; official `PointfootHardwareNode.cpp` | FCR estop in sim; `L1 + X` code read. | Topic zeroing and code semantics understood. | PASS for software stop; UNVERIFIED for physical stopping distance. |
| Damping | The only observed damping evidence is the real hardware motor switch / hardware action log `Motor in damping mode`. | `docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md`; `docs/TRON1_REAL_TEST_STEP_CHECKLIST.md` | Human-triggered physical action on robot, not Gazebo. | Recorded as physical evidence; not automated. | PASS as observed hardware path; not a Gazebo result. |
| Simulation reset state | Reset can change contact/pose conditions and must not be treated as a clean safety proof unless command layer and graph gates are rerun. | Acceptance/gate scripts | Relaunch sim, rerun acceptance, check dirty processes and graph. | No new runtime change in this freeze. | UNVERIFIED for physical stop; PASS only after rerun. |
| Command source disappearance | Same as B/D. | Same as B/D. | Stop the command publisher or kill the source. | Covered at desired-command layer. | PASS at topic/controller-intent layer. |

## Accepted Fixes This Freeze

- Keep FCR command route isolated: `/fcr/cmd_vel_stamped -> tron1_safety_limiter -> /fcr_tron/cmd_vel`.
- Keep bare `/cmd_vel` out of the FCR real path.
- Keep `enable_motion=false` and `allow_tron_follow_motion=false` as launch defaults.
- Keep route/ping/network checks as read-only non-motion checks.
- Record the limitation that Gazebo zero-drift remains a blocker for real ground motion claims.
- Record that remote left/right stick e-stop is not yet verified to override FCR runtime commands.

## Rejected Fixes

- Do not raise Gazebo damping/friction just to hide drift.
- Do not call zero desired velocity "physical stop".
- Do not call `PASS_FOR_STAGED_AIRBORNE_TEST` a 100% safety guarantee.
- Do not use the TRON hand controller joystick axes as an FCR command source.
- Do not enter ground real motion from this freeze.

## Freeze Status

```text
Gazebo safety issue: converged, not fully solved.
Zero-command drift: known blocker carried forward.
Command timeout / lost command: PASS at command-intent layer.
Node crash: mode-manager crash covered; limiter hard crash still relies on official watchdog and physical stop.
Damping: physical path observed outside Gazebo; not proven by sim.
Real-motion status: paused.
```
