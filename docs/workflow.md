# Development Workflow

## Recommendation

Long-term best workflow:

```text
Windows: Codex, documentation, code review, patch preparation
Ubuntu 22.04: ROS 2 build, Gazebo, SDK, sensors, TRON1 hardware
Git: transfer boundary between the two
```

If Ubuntu network can reliably access GitHub, VS Code, and OpenAI/Codex, then move Codex directly onto Ubuntu. That is the simplest daily workflow.

## Option A: Ubuntu + VS Code + Codex

Best if network works.

- Configuration difficulty: medium, mostly network and login.
- Stability: high after network is fixed.
- Codex experience: best, because Codex sees the real ROS workspace and build errors directly.
- ROS 2 experience: best.
- Gazebo GUI: best on native Ubuntu.
- USB/Serial/CAN/Camera/TRON1: best on native Ubuntu.

Needed categories:

```text
Ubuntu network
GitHub access
OpenAI/Codex access
VS Code login/extensions
ROS 2 workspace
hardware permissions
```

## Option B: Windows Codex, Ubuntu Build Machine

Useful immediately.

```text
Windows edits -> Git commit/push or copy patch -> Ubuntu pull/apply -> build/test -> paste logs back
```

- Configuration difficulty: low.
- Stability: medium.
- Codex experience: good for code and docs, weaker for live ROS debugging.
- ROS 2 experience: good on Ubuntu, but feedback loop is slower.
- Hardware debugging: workable but copy-paste heavy.

Use this until Ubuntu can run Codex reliably.

## Option C: Windows VS Code Remote SSH to Ubuntu

Potentially excellent, but depends on where the AI extension runs.

- VS Code UI runs on Windows.
- The workspace, terminal, compiler, ROS 2 tools, and extensions often run on the Remote Host.
- If Codex extension or CLI needs network from the remote host, Ubuntu still needs OpenAI access.
- If Codex is controlled from Windows but edits remote files through VS Code, Windows network may be enough; this must be tested with the actual Codex integration.

This is a strong backup if Ubuntu desktop login is inconvenient, but it may not solve OpenAI network access if the extension executes remotely.

## Option D: WSL2 / Docker / Dev Container

Good for repeatable compile experiments, not ideal as the first hardware path.

- WSL2: convenient on Windows, but ROS 2 networking, Gazebo GUI, USB, CAN, cameras, and real robot networking add complexity.
- Docker: good for clean builds and CI, painful for GUI, USB, CAN, and camera unless carefully configured.
- Dev Container: good after dependencies are known, not before first hardware bringup.

Use these later as reproducibility tools, not as the primary robot development environment.

## Practical Decision

Start with Option B now, because it works immediately. Aim for Option A as the long-term target. Keep Option C as a fallback if Ubuntu desktop network or login remains annoying but SSH is stable.
