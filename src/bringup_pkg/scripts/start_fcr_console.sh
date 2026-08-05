#!/usr/bin/env bash
set -Eeuo pipefail

# Start the complete real-hardware stack in the background and keep the
# foreground terminal exclusively for the integrated operator console.
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WORKSPACE="${FCR_ROS2_WS:-$HOME/ros2_ws}"

set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"
set -u

bringup_pid=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$bringup_pid" ]] && kill -0 "$bringup_pid" 2>/dev/null; then
    echo
    echo "Stopping FCR hardware stack..."
    kill -INT "$bringup_pid" 2>/dev/null || true
    wait "$bringup_pid" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

ros2 run bringup_pkg start_fcr.sh "$@" &
bringup_pid=$!

echo "Waiting for command mux and operator services..."
deadline=$((SECONDS + 45))
while ((SECONDS < deadline)); do
  if ! kill -0 "$bringup_pid" 2>/dev/null; then
    wait "$bringup_pid"
    exit $?
  fi
  if ros2 node list 2>/dev/null | grep -qx '/command_mux' &&
     ros2 action list 2>/dev/null | grep -qx '/manual_jog/execute'; then
    break
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -qx '/command_mux'; then
  echo "ERROR: command_mux did not become ready within 45 seconds." >&2
  exit 1
fi
if ! ros2 action list 2>/dev/null | grep -qx '/manual_jog/execute'; then
  echo "ERROR: MANUAL_JOG action did not become ready within 45 seconds." >&2
  exit 1
fi

ros2 run teleop_control_pkg fcr_mode_console
