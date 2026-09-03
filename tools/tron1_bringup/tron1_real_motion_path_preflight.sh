#!/usr/bin/env bash
# Read-only preflight for the TRON1 real-motion path.
# This script never publishes velocity commands.

set -u

TRON_IP="${TRON_IP:-10.192.1.2}"
JETSON_IP="${JETSON_IP:-172.31.178.242}"
FCR_WS="${FCR_WS:-$HOME/follow_ws}"
LIMX_WS="${LIMX_WS:-$HOME/limx_ws}"

section() {
  printf '\n===== %s =====\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

section "Inputs"
echo "TRON_IP=$TRON_IP"
echo "JETSON_IP=$JETSON_IP"
echo "FCR_WS=$FCR_WS"
echo "LIMX_WS=$LIMX_WS"

section "Network route checks"
run ip -brief addr
run ip route get "$TRON_IP"
run ping -c 1 -W 1 "$TRON_IP"

section "ROS environment"
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [ -f "$FCR_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$FCR_WS/install/setup.bash"
fi
if [ -f "$LIMX_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$LIMX_WS/install/setup.bash"
fi
set -u

run ros2 pkg executables robot_platform_pkg
run ros2 pkg executables robot_hw

section "Safe launch defaults"
export ROBOT_TYPE="${ROBOT_TYPE:-WF_TRON1A}"
export RL_TYPE="${RL_TYPE:-isaacgym}"
run ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py --show-args
run ros2 launch robot_hw pointfoot_hw.launch.py --show-args

section "If graph is already running"
run ros2 topic info -v /fcr_tron/cmd_vel
run ros2 topic info -v /fcr/cmd_vel_stamped
run ros2 topic info -v /cmd_vel

section "Result guide"
echo "PASS for low-speed prep requires:"
echo "  1) TRON_IP route does not go through a proxy/TUN interface."
echo "  2) ping or SDK-level connection to TRON succeeds."
echo "  3) fcr_tron_jetson_comm.launch.py keeps start_tron_hw=false and enable_motion=false by default."
echo "  4) /fcr_tron/cmd_vel has exactly one publisher: tron1_safety_limiter."
echo "  5) /cmd_vel has no TRON controller subscriber during FCR tests."
echo
echo "This preflight is read-only: no real motion command was sent."
