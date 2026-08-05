#!/usr/bin/env bash
set -Eeuo pipefail

# Start the complete real-hardware stack in the background and keep the
# foreground terminal exclusively for the integrated operator console.
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WORKSPACE="${FCR_ROS2_WS:-$HOME/ros2_ws}"
STARTUP_TIMEOUT_SEC="${FCR_CONSOLE_STARTUP_TIMEOUT_SEC:-120}"

if [[ ! "$STARTUP_TIMEOUT_SEC" =~ ^[0-9]+$ ]] ||
   ((STARTUP_TIMEOUT_SEC < 10)); then
  echo "ERROR: FCR_CONSOLE_STARTUP_TIMEOUT_SEC must be an integer >= 10." >&2
  exit 2
fi

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

# Authenticate before the timeout starts.  The hardware bringup configures
# SocketCAN with sudo; prompting from the background job otherwise consumes
# the old 45-second readiness window and intermittently prevents the console
# from ever being launched.
if ! sudo -v; then
  echo "ERROR: sudo authentication failed; hardware bringup was not started." >&2
  exit 1
fi

ros2 run bringup_pkg start_fcr.sh "$@" &
bringup_pid=$!

echo "Waiting up to ${STARTUP_TIMEOUT_SEC}s for command mux and operator services..."
deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
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
  echo "ERROR: command_mux did not become ready within ${STARTUP_TIMEOUT_SEC}s." >&2
  exit 1
fi
if ! ros2 action list 2>/dev/null | grep -qx '/manual_jog/execute'; then
  echo "ERROR: MANUAL_JOG action did not become ready within ${STARTUP_TIMEOUT_SEC}s." >&2
  exit 1
fi

echo "Operator services ready; starting integrated console."
ros2 run teleop_control_pkg fcr_mode_console
