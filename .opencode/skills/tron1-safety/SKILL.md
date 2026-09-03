---
name: tron1-safety
description: Enforce simulation-first and human-approved safety rules for any TRON1 movement, cmd_vel, velocity, motor, joint, SDK command, timeout, watchdog, or e-stop task.
compatibility: opencode
metadata:
  project: followrobot
---

## Mandatory Rule

Any task involving TRON1 movement, `/cmd_vel`, `/fcr_tron/cmd_vel`, velocity, acceleration, motor, joint, SDK command, robot command, watchdog, timeout, or e-stop is safety-critical.

Default to:

```text
Simulation
  -> Safety Review
  -> Low-speed Test Plan
  -> Human Approval
  -> Real Robot
```

## Execution Boundary

OpenCode and local models must not automatically execute real TRON1 movement commands.

They may:

- Inspect code and docs.
- Generate simulation commands.
- Generate real-robot commands for human review.
- Review safety limits.
- Run non-motion checks.

They must not:

- Auto-run real robot movement.
- Bypass `tron1_safety_limiter`.
- Publish directly to TRON1 bare `/cmd_vel`.
- Increase speed or acceleration limits without explicit human approval.
- Send joint, motor, or SDK commands to hardware unless the human explicitly authorizes that exact test.

## Required Labels

Every command proposal must be labeled as one of:

- `SIMULATION`
- `NON-MOTION HARDWARE CHECK`
- `REAL ROBOT - HUMAN APPROVAL REQUIRED`

