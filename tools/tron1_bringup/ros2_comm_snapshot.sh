#!/usr/bin/env bash
# Compact ROS 2 graph snapshot for Jetson + RS2 + depth-camera bring-up.

set -u

section() {
  printf '\n===== %s =====\n' "$1"
}

run_ros() {
  printf '\n$ %s\n' "$*"
  bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; $*" 2>&1 || true
}

section "ROS environment"
run_ros "which ros2"
run_ros "printenv | grep -E 'ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|AMENT_PREFIX_PATH' || true"

section "Nodes"
run_ros "ros2 node list"

section "Topics of interest"
run_ros "ros2 topic list | grep -E 'gimbal|camera|depth|perception|cmd_vel|fcr_tron|estop|remote_control|joint_states|diagnostics|rosout' || true"

section "Gimbal"
run_ros "ros2 lifecycle get /gimbal_driver"
run_ros "ros2 param get /gimbal_driver can_interface"
run_ros "ros2 param get /gimbal_driver control_mode"
run_ros "ros2 topic echo /gimbal/status --once"
run_ros "ros2 topic info -v /cmd_gimbal"
run_ros "ros2 topic info -v /cmd_gimbal_nudge"

section "Depth camera"
run_ros "timeout 8s ros2 topic hz /camera/depth/image_raw"
run_ros "ros2 topic echo /camera/depth/camera_info --once"

section "TRON safety topics"
run_ros "ros2 param get /tron1_safety_limiter enable_motion"
run_ros "ros2 topic info -v /fcr_tron/cmd_vel"
run_ros "ros2 topic echo /fcr_tron/cmd_vel --once"

section "Done"
echo "Paste this snapshot into Codex when a stage fails or looks suspicious."
