# followrobot Agent Rules

This is a ROS 2 robot project for real hardware. Treat it as safety-sensitive engineering work, not a generic software repository.

## Current Project State

- The stable working chain is Jetson + Sony camera + DJI RS2 gimbal. Vision, tracking, and gimbal control can already perform real follow shooting.
- The active development focus is TRON1 EDU wheeled-foot base integration.
- Keep the working Sony/RS2 tracking path stable unless the task explicitly targets it.
- The user fork is `followrobot` at `https://github.com/Miya555orz/followrobot.git`. Upstream `origin` points to the older `cuiangA/fcr_ros2_3` repository.

## Architecture Rules

- Do not couple perception or tracking directly to a specific base.
- Base-specific behavior belongs behind a base interface, adapter, launch file, or config file.
- The intended TRON1 chain is:

```text
Follow Controller
  -> Unified Base Command
  -> base_interface / adapter
  -> tron1_safety_limiter
  -> /fcr_tron/cmd_vel
  -> TRON1 official ROS2 controller / SDK
  -> TRON1 EDU
```

- Never let TRON1 consume the legacy bare `/cmd_vel` path. TRON1 must consume the safety-limited `/fcr_tron/cmd_vel` topic.
- Do not send joint torque, joint position, motor command, or raw SDK command from followrobot unless the task explicitly requests a low-level SDK experiment and it has passed a safety review.

## Safety Rules

- Simulation first for any base-motion change.
- Safety review is required for changes involving velocity, acceleration, watchdog, timeout, e-stop, `/cmd_vel`, SDK command, motor command, or joint command.
- Real TRON1 commands must not be executed automatically by an AI fallback agent.
- Any real robot command must be generated for human review, with explicit SIMULATION or REAL ROBOT labeling.
- Default real-robot posture is motion disabled. Keep `enable_motion=false` unless a human explicitly enables a staged test.
- If a test mentions TRON1 and does not explicitly say simulation, assume it is unsafe to auto-execute.

## Build And Test

- Inspect before modifying: `git status --short --branch`, relevant files, and existing docs.
- Preserve user changes. Do not reset, checkout, or delete unrelated work.
- For ROS 2 builds, source the environment first:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
```

- Prefer focused builds for changed packages:

```bash
cd /home/miya/follow_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 --packages-select <packages>
```

- For TRON1 official workspace checks:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
```

## OpenCode Role Split

- OpenCode may handle low-risk tasks: docs, comments, single-file refactors, launch/config cleanup, package metadata, small tests, and code search.
- Codex should handle architecture decisions, complex ROS 2 integration, safety-critical review, TRON1 control decisions, and large refactors.
- Reviewer and Challenger passes are required before merging safety-critical robot changes.
- After a major stage, update `README.md`, `docs/HANDOFF.md`, and relevant docs under `docs/ai/`.

