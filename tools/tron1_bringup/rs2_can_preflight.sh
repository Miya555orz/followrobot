#!/usr/bin/env bash
# Read-only DJI RS2 CAN preflight. It inspects SocketCAN state and listens for
# RS2 response frames. It does not send motion commands.

set -u

CAN_IF="${1:-${JETSON_CAN_INTERFACE:-can0}}"
LISTEN_SECONDS="${RS2_CAN_LISTEN_SECONDS:-5}"

section() {
  printf '\n===== %s =====\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

section "CAN interface"
run ip -details -statistics link show "$CAN_IF"

section "Driver"
if command -v ethtool >/dev/null 2>&1; then
  run ethtool -i "$CAN_IF"
else
  echo "ethtool not installed; skip driver report"
fi

section "CAN tools"
run bash -lc "command -v candump || true"
run bash -lc "command -v cansend || true"

section "RS2 receive-frame listen"
echo "Listening for up to ${LISTEN_SECONDS}s on ${CAN_IF}, filter 0x222."
echo "No frames here is not a final failure: RS2 may stay silent until queried by gimbal_driver."
if command -v candump >/dev/null 2>&1; then
  timeout "${LISTEN_SECONDS}s" candump -L "${CAN_IF},222:7FF" 2>&1 || true
else
  echo "candump not found; install can-utils first."
fi

section "After-listen counters"
run ip -details -statistics link show "$CAN_IF"

section "Next"
cat <<EOF
If ${CAN_IF} is UP, ERROR-ACTIVE, bitrate 1000000, and counters do not explode:
  source /opt/ros/humble/setup.bash
  source ~/follow_ws/install/setup.bash
  ros2 launch robot_platform_pkg gimbal_bringup.launch.py use_sim:=false can_interface:=${CAN_IF} control_mode:=incremental_position
EOF
