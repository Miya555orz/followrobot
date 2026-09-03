#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="/home/miya/follow_ws/src/fcr_ros2_3"
OPENCODE_BIN="${OPENCODE_BIN:-/home/miya/.opencode/bin/opencode}"

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 \"task for OpenCode\""
  exit 2
fi

if [ ! -x "$OPENCODE_BIN" ]; then
  echo "OpenCode executable not found: $OPENCODE_BIN"
  echo "Install or set OPENCODE_BIN=/path/to/opencode"
  exit 127
fi

TASK="$*"

cd "$PROJECT_ROOT"

PROMPT=$(cat <<EOF
PROJECT:
followrobot

WORKDIR:
$PROJECT_ROOT

READ FIRST:
- AGENTS.md
- docs/ai/PROJECT_CONTEXT.md
- docs/ai/ARCHITECTURE.md
- docs/ai/CURRENT_STATUS.md
- docs/ai/SAFETY_RULES.md
- docs/ai/TRON1_CONTEXT.md
- docs/ai/HARNESS_ROUTING.md

TASK:
$TASK

CONSTRAINTS:
- Inspect git status and relevant files before modifying.
- Preserve existing behavior unless explicitly asked.
- Keep the verified Jetson + Sony + DJI RS2 follow chain stable unless the task explicitly touches it.
- Do not modify /etc, systemd, udev, kernel modules, Jetson network, TRON1 network, or CAN setup.
- Do not execute real TRON1 movement commands.
- Do not publish directly to TRON1 bare /cmd_vel.
- If the task involves TRON1 movement, velocity, acceleration, SDK command, motor command, joint command, watchdog, timeout, or e-stop, treat it as safety-critical and produce simulation-only steps unless human approval is explicit.
- Run focused tests when safe; otherwise explain why tests were not run.
- Report changed files and verification.
EOF
)

if [ -n "${OPENCODE_MODEL:-}" ]; then
  exec "$OPENCODE_BIN" run --dir "$PROJECT_ROOT" --model "$OPENCODE_MODEL" "$PROMPT"
fi

exec "$OPENCODE_BIN" run --dir "$PROJECT_ROOT" "$PROMPT"
