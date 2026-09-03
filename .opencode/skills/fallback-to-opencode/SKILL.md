---
name: fallback-to-opencode
description: Use OpenCode as a low-risk fallback when Codex quota, rate limit, or task size makes delegation useful for this followrobot ROS2 project.
compatibility: opencode
metadata:
  project: followrobot
---

## Purpose

Use this skill when Codex is near token quota, hits a rate limit, or the task is suitable for a cheaper/local coding agent.

Good OpenCode tasks:

- Single-file refactors
- README and handoff edits
- Comments and small docs
- CMakeLists.txt and package.xml cleanup
- Launch/config edits
- Focused unit tests
- Code search and summaries
- Small bug fixes with clear acceptance criteria

Keep Codex in charge of architecture, TRON1 control-level decisions, high-risk safety changes, and real robot test planning.

## Delegation Prompt Shape

Do not pass the raw user prompt alone. Wrap it with project context:

```text
PROJECT:
followrobot

WORKDIR:
/home/miya/follow_ws/src/fcr_ros2_3

TASK:
<specific task>

CONSTRAINTS:
- Inspect git status and relevant files before modifying.
- Preserve existing behavior unless explicitly asked.
- Do not touch real robot, Jetson network, TRON1 network, CAN, udev, systemd, or /etc.
- Do not execute real TRON1 movement commands.
- Run focused tests or explain why they were not run.
- Report changed files and verification.
```

## Command

Prefer the project wrapper:

```bash
/home/miya/follow_ws/src/fcr_ros2_3/scripts/opencode-fallback.sh "TASK"
```

If a specific model is configured:

```bash
OPENCODE_MODEL=provider/model /home/miya/follow_ws/src/fcr_ros2_3/scripts/opencode-fallback.sh "TASK"
```

