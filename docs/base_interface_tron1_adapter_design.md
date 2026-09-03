# Base Interface + TRON1 Adapter Design

Purpose: keep FCR perception/tracking/follow logic independent from the physical base. TRON1-specific behavior must live only in adapter/limiter/bringup layers.

## Target architecture

```text
Perception / Tracking / Follow
          |
          v
Unified Base Interface
          |
   +------+------+
   |             |
Omni Adapter   TRON1 Adapter
   |             |
Old Base       tron1_safety_limiter
                 |
                 v
              /fcr_tron/cmd_vel
                 |
                 v
              TRON1 official controller
```

## Current practical interface

Use the already-tested command type as the first implementation target:

```text
/fcr/cmd_vel_stamped
geometry_msgs/msg/TwistStamped
```

Why:

- FCR already uses `TwistStamped` through command mux.
- TRON1 official controller consumes `Twist`.
- `tron1_safety_limiter` already bridges stamped FCR commands to `/fcr_tron/cmd_vel`.
- This avoids creating new message types before the hardware safety path is proven.

Future richer API can be added later as `vision_servo_msgs/BaseCommand`, but do not block TRON1 safety bringup on it.

## Topic ownership

| Layer | Topic | Type | Owner |
| --- | --- | --- | --- |
| upper follow output | `/fcr/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | command mux / base interface |
| TRON limiter input | `/fcr/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | `tron1_safety_limiter` subscriber |
| TRON limiter output | `/fcr_tron/cmd_vel` | `geometry_msgs/Twist` | only `tron1_safety_limiter` publisher |
| TRON official control | `/fcr_tron/cmd_vel` | `geometry_msgs/Twist` | official `robot_hw_node` subscriber |
| forbidden direct path | `/cmd_vel` | `geometry_msgs/Twist` | not allowed for FCR/TRON integration |

## Package boundary

Recommended near-term layout:

```text
src/servo_control_pkg/
  follow controller, gimbal/base error split

src/robot_platform_pkg/
  tron1_safety_limiter_node
  tron_cmd_adapter_node
  platform status and hardware-facing adapters

src/bringup_pkg/
  launch composition and mode selection
```

Do not put TRON1-specific code in:

- `src/perception_pkg`
- target detection / tracking logic
- Sony / Orbbec camera nodes
- generic visual servo math

## Adapter responsibilities

`tron1_adapter` may know:

- TRON1 cannot use lateral velocity for early tests.
- TRON1 yaw should be slow and long-horizon only.
- TRON1 requires `/fcr_tron/cmd_vel`, not bare `/cmd_vel`.
- `enable_motion=false` is the real-robot default.
- lost target must produce zero base command.

`tron1_adapter` must not:

- bypass `tron1_safety_limiter`;
- publish directly to `/cmd_vel`;
- decide visual target identity;
- command full-speed follow;
- hide safety failures from the upper mode manager.

## Control split for following

The current safest follow split is:

```text
RS2 gimbal:
  short-term small-angle target centering

TRON1 base:
  slow long-term yaw recentering
  slow distance compensation only after real low-speed acceptance
```

Avoid two independent controllers chasing the same pixel error. The base should consume a filtered, low-frequency residual error after the gimbal has done fast stabilization.

## Launch policy

Launch should select base backend by parameter, not by editing perception/follow code.

Near-term modes:

```text
base_backend:=none
  perception + gimbal only

base_backend:=omni
  legacy base path

base_backend:=tron1_sim
  TRON1 topic safety in simulation

base_backend:=tron1_real
  real TRON1 path, default enable_motion=false
```

For `tron1_real`:

```text
enable_motion=false
max_linear_x=0.02~0.03
max_linear_y=0.0
max_angular_z=0.05~0.10
```

## Acceptance before implementation

Before wiring follow into TRON1:

- [ ] `/fcr_tron/cmd_vel` has only one publisher: `tron1_safety_limiter`.
- [ ] official controller subscribes `/fcr_tron/cmd_vel`.
- [ ] `enable_motion=false` makes output zero under all upstream commands.
- [ ] timeout returns output to zero.
- [ ] estop returns output to zero.
- [ ] limiter shutdown sends zero burst.
- [ ] publisher/lost-limiter behavior is documented.
- [ ] hardware damping/stop path is understood.

## Gazebo caveat

`WF_TRON1A + isaacgym` Gazebo currently drifts at zero command and pure yaw commands produce lateral translation. `RL_TYPE=isaaclab` was tested and drifted more. Lightweight controller and URDF friction experiments were withdrawn.

Therefore:

- use Gazebo to validate launch, topic wiring, limiter, timeout, estop, and command signs;
- do not use Gazebo pose hold as the pass/fail gate for real robot safety;
- do not tune FCR follow behavior around this Gazebo drift.

## Next implementation step

Implement the smallest useful interface first:

1. Keep `/fcr/cmd_vel_stamped` as the unified command topic.
2. Make `bringup_pkg` select whether this topic is connected to no base, old omni base, or TRON1 limiter.
3. Add a `base_backend` launch argument.
4. Keep `tron1_safety_limiter` mandatory for every TRON1 mode.
5. Add acceptance scripts that prove topic ownership and safety outputs before any real motion.
