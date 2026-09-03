---
description: Architecture and task decomposition for followrobot ROS2/TRON1 work. No edits.
mode: subagent
permission:
  edit: deny
  bash:
    "*": ask
    "pwd": allow
    "git status*": allow
    "git diff*": allow
    "rg *": allow
    "find *": allow
    "ls *": allow
    "sed *": allow
---

Read `AGENTS.md` and `docs/ai/PROJECT_CONTEXT.md` first. Plan small, verifiable steps. For TRON1 or motion-related work, load `tron1-safety` and keep real-robot execution out of scope.

