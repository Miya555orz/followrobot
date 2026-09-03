#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/miya/follow_ws/src/fcr_ros2_3"
FOLLOW_WS="/home/miya/follow_ws"
LIMX_WS="/home/miya/limx_ws"

export ROBOT_TYPE="${ROBOT_TYPE:-WF_TRON1A}"
export RL_TYPE="${RL_TYPE:-isaacgym}"
export FCR_TRON_CMD_VEL_TOPIC="${FCR_TRON_CMD_VEL_TOPIC:-/fcr_tron/cmd_vel}"
export FCR_TRON_CMD_VEL_TIMEOUT_SEC="${FCR_TRON_CMD_VEL_TIMEOUT_SEC:-0.25}"

source /opt/ros/humble/setup.bash
if [ -f "$FOLLOW_WS/install/setup.bash" ]; then
  source "$FOLLOW_WS/install/setup.bash"
fi
if [ -f "$LIMX_WS/install/setup.bash" ]; then
  source "$LIMX_WS/install/setup.bash"
fi

cd "$PROJECT_ROOT"
python3 tools/tron1_bringup/tron1_safe_mode_acceptance.py "$@"
