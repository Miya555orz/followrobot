# Safety Rules

TRON1 EDU is a real wheeled-foot robot. Treat base motion as safety-critical.

## Never Bypass

- Do not let TRON1 subscribe to bare `/cmd_vel`.
- Do not bypass `tron1_safety_limiter`.
- Do not auto-execute real robot movement commands from OpenCode, local models, scripts, or fallback agents.

## Required Gate

```text
Simulation PASS
SDK communication PASS
E-stop PASS
Timeout PASS
Velocity clamp PASS
Command-loss stop PASS
Operator ready
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

## Command Labels

All proposed robot commands must be labeled:

- `SIMULATION`
- `NON-MOTION HARDWARE CHECK`
- `REAL ROBOT - HUMAN APPROVAL REQUIRED`

