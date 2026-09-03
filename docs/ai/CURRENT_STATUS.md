# Current Status

Last aligned on 2026-09-03.

## Distance To The Next Milestones

- Jetson controls TRON1: about halfway to a safe acceptance test. The FCR command path and PC-side TRON Ethernet/SDK path are proven, but Jetson <-> TRON Ethernet, Jetson-side graph checks, simulation stop behavior, and low-speed real acceptance are still open.
- Jetson controls RS2 gimbal follow: nearly ready. Sony UVC -> perception/tracking -> RS2 over `can1` has already worked on Jetson; CRSDK remains optional/unverified.
- Jetson controls RS2 follow + TRON1 base: not ready for real full-chain following. Keep this in simulation until TRON1 stop/damping/controller behavior is familiar and the base is limited to slow long-term yaw/distance correction rather than chasing the same error as the gimbal.

Current posture: TRON1 real motion is paused. Continue remote-controller familiarization and Gazebo simulation first.

## Verified

- Jetson + Sony camera + DJI RS2 gimbal can work together.
- Vision -> tracking -> gimbal control can perform real follow shooting.
- The active followrobot checkout is `/home/miya/follow_ws/src/fcr_ros2_3`.
- The latest observed commit was `b680058 Record final Sony RS2 follow tuning`.
- TRON1 official workspace exists at `/home/miya/limx_ws`.
- TRON1 ROS packages are visible under the local ROS 2 Humble environment.
- TRON1 official `robot_hw` launch exposes `fcr_cmd_vel_topic`; the local official workspace still contains the FCR override patch.
- FCR-side `robot_platform_pkg`, `teleop_control_pkg`, and `bringup_pkg` build successfully on this machine.
- TRON1-side `robot_controllers` and `robot_hw` build successfully on this machine.
- Simulation-only safety limiter smoke test passed: with `enable_motion=false`, oversized upstream commands still produced zero `/fcr_tron/cmd_vel`; with `enable_motion=true`, oversized commands were clamped and returned to zero after timeout.
- Current DJI RS2 bench wiring uses external USB-CAN `can1`; TRON1 communication launch defaults have been aligned to `can1`.
- PC-side Jetson network preflight exists at `tools/tron1_bringup/pc_jetson_network_preflight.sh`.
- TRON1 read-only real-motion path preflight exists at `tools/tron1_bringup/tron1_real_motion_path_preflight.sh`.
- TRON1 migration gate report exists at `docs/tron1_migration_gate_report_2026-09-03.md`.
- TRON1 official sim launch now defaults `start_steering_gui=false`, so FCR safety-chain tests no longer auto-start `rqt_robot_steering`.

## Current Work

TRON1 EDU wheeled-foot base secondary development and full system integration.

## Important Caveats

- The official TRON1 ROS2 docs target ROS 2 Iron, while this local machine currently uses ROS 2 Humble.
- `/home/miya/limx_ws/src/tron1-rl-deploy-ros2` contains a local patch that lets the official controller subscribe to `/fcr_tron/cmd_vel`.
- Ollama was not installed when this AI environment was created, so local model routing is prepared but not active.
- Real TRON1 movement is not part of OpenCode fallback dry runs.
- A TRON1 Gazebo simulation process was already running during the 2026-09-03 environment check; no real robot hardware movement was commanded.
- Current PC network blocker: Ethernet `enp0s31f6` has `carrier=0`, and route to Jetson `172.31.178.242` is captured by Mihomo table 2022. Fix physical link first; if carrier is present but route is still captured, use the preflight script's `--fix-route` mode.
- TRON1 PC-side Ethernet blocker was cleared on 2026-09-03: `10.192.1.2` became reachable through `enp0s31f6`, and official `pointfoot_node` connected to the real robot.
- TRON1 remote controller axes/buttons were observed through a read-only SDK monitor. `L1 + Y/triangle` activated `WheelfootController`.
- A physical motor switch/hardware action produced `Motor in damping mode`; official `pointfoot_node` then stopped the controller and exited. `L1 + X` should be treated as software stop/abort, not damping/torque release.
- Real TRON1 motion is paused because the first controller activation felt too strong/fast. Continue in Gazebo/simulation and remote-controller familiarization before any more real motion.
- Current TRON1 safety state: FCR limiter clamp/acceleration/timeout/estop and clean-shutdown zero-burst pass at topic-output level. The official controller watchdog clears stale velocity intent; however `WF_TRON1A + isaacgym` Gazebo still drifts at zero command. A hard hold/damping safe-stop experiment destabilized the model and was withdrawn. Do not run real TRON1 motion until controller/SDK/hardware stop behavior is verified.
