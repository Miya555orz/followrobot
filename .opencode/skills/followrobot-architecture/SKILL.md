---
name: followrobot-architecture
description: Preserve the current followrobot architecture while integrating TRON1 through base interfaces, adapters, launch files, and config.
compatibility: opencode
metadata:
  project: followrobot
---

## Stable Chain

The currently verified chain is:

```text
Sony camera
  -> Jetson
  -> perception / tracking
  -> follow / visual servo
  -> DJI RS2 gimbal
```

Do not refactor this chain unless the task explicitly targets it.

## Target Base Architecture

TRON1 integration should follow:

```text
Follow Controller
  -> Unified Base Command
  -> base_interface
  -> tron1_adapter / safety limiter
  -> TRON1 ROS2 controller / SDK
  -> TRON1 EDU
```

Changing base platforms should mostly replace:

- adapter
- driver/backend
- launch
- config

It should not require perception, tracking, or high-level follow logic rewrites.

## Design Rule

TRON1-specific state machines, command mapping, velocity clamps, acceleration limiting, watchdogs, timeout handling, and state validation belong in the adapter/backend layer, not in perception or tracking.

