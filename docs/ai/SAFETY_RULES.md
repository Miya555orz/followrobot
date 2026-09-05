# Safety Rules

TRON1 EDU is a real wheeled-foot robot. Treat base motion as safety-critical.

## Never Bypass

- Do not let TRON1 subscribe to bare `/cmd_vel`.
- Do not bypass `tron1_safety_limiter`.
- Do not auto-execute real robot movement commands from OpenCode, local models, scripts, or fallback agents.
- Do not treat the TRON1 guide's left/right stick "global e-stop" as verified for FCR runtime command override until `docs/TRON1_FCR_REMOTE_ESTOP_SEMANTICS_2026-09-05.md` is upgraded from `[UNVERIFIED]`.

## Required Gate

```text
Simulation PASS
SDK communication PASS
E-stop PASS
Timeout PASS
Velocity clamp PASS
Command-loss stop PASS
Operator ready
Runbook step PASS
  -> REAL ROBOT ENABLED
```

## Review Required

Any change involving these words or systems needs reviewer/challenger review:

- velocity
- acceleration
- e-stop
- watchdog
- command timeout
- `/cmd_vel`
- `/fcr_tron/cmd_vel`
- SDK command
- motor command
- joint command
- remote e-stop
- left/right stick press

## Current Freeze

As of 2026-09-05, real TRON1 motion remains paused. Jetson to TRON1 Ethernet network-only is PASS, but Gazebo zero-command drift and FCR runtime remote e-stop coverage remain unresolved safety items. Future real commands must be entered through `docs/TRON1_REAL_TEST_RUNBOOK.md`, not invented ad hoc.

## Command Labels

All proposed robot commands must be labeled:

- `SIMULATION`
- `NON-MOTION HARDWARE CHECK`
- `REAL ROBOT - HUMAN APPROVAL REQUIRED`
