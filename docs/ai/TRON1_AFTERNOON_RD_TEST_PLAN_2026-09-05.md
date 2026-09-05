# TRON1 Afternoon R&D and Test Plan - 2026-09-05

Scope: afternoon work after PC USB SSH and Jetson-to-TRON1 Ethernet network-only PASS. Real TRON1 motion remains paused.

## Safety Boundary

Allowed:

- Documentation and review updates.
- Read-only network checks.
- Static code review and script linting.
- Simulation-only work.
- Physical checklist preparation without powering or commanding motion.

Forbidden:

- `ros2 launch robot_hw ...`
- `ros2 run robot_hw ...`
- `ros2 topic pub` to any velocity topic.
- `enable_motion:=true`.
- Remote-controller `L1 + Y/triangle` controller activation.
- Any ground motion in the small office.

## Afternoon Schedule

### 13:30-14:00 - Archive and Review Current Evidence

- Save PC USB SSH quickstart evidence.
- Save Jetson-to-TRON1 network-only PASS evidence.
- Ask OpenCode to review the new evidence and confirm no P0/P1 wording risk.

Exit condition:

```text
README / CURRENT_STATUS / HANDOFF all agree:
PC->Jetson USB SSH works.
Jetson->TRON1 Ethernet network-only works.
Real motion is still paused.
```

### 14:00-14:45 - Make the Network Setup Repeatable

- Decide whether to keep temporary `ip route replace` commands or add a documented NetworkManager profile.
- If changing NetworkManager, keep it route-only and document rollback.
- Do not make this an auto-start controller path.

Exit condition:

```text
ip route get 10.192.1.2
```

still shows:

```text
10.192.1.2 dev enP8p1s0 src 10.192.1.200
```

### 14:45-15:45 - Simulation-Only Safety Work

- Re-run or inspect the Gazebo zero-drift blocker in simulation only.
- Keep documenting that Gazebo PASS is not real-motion permission.
- Review A-10 items and separate what can be checked on the bench from what needs an open protected space.

Exit condition:

```text
No real robot movement.
No controller activation.
Updated notes identify exactly what remains blocked before any motion.
```

### 15:45-16:30 - Step 0/Step 1 Checklist Prep

- Prepare the wording for stand/架空 Step 1.
- Confirm the checklist requires physical stop reachability, damping behavior, and controller watchdog understanding before motion.
- Keep speed limits and command path unchanged unless a separate reviewed change is planned.

Exit condition:

```text
Step 1 is prepared as a future protected-space test, not an office test.
```

### 16:30-17:00 - End-of-Day Handoff

- Update `docs/ai/CURRENT_STATUS.md`.
- Update `docs/HANDOFF.md`.
- Commit and push only documentation or read-only tooling changes.
- Prepare the next review prompt.

## Next Green Light Definition

The current green light is only:

```text
Network-only connectivity is proven.
```

It does not mean:

```text
Ground motion is allowed.
The official controller may be activated.
The robot can follow a person.
```
