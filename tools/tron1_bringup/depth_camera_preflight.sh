#!/usr/bin/env bash
# Orbbec/Gemini depth-camera preflight for Jetson. Safe/read-only: it checks
# USB, ROS packages, launch visibility, and currently published depth topics.

set -u

section() {
  printf '\n===== %s =====\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

section "USB devices"
run lsusb

section "Workspace packages"
run bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; ros2 pkg list | grep -E 'orbbec_camera$|orbbec_camera_msgs|perception_pkg|vision_servo_msgs' || true"

section "Launch files"
run bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; ros2 launch orbbec_camera gemini2.launch.py --show-args | sed -n '1,80p'"
run bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; ros2 launch orbbec_camera gemini_330_series_low_cpu.launch.py --show-args | sed -n '1,80p'"

section "Currently published camera topics"
run bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; ros2 topic list | grep -E '/camera|depth|image|camera_info' || true"

section "Depth topic sampling if camera is already running"
run bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; timeout 8s ros2 topic hz /camera/depth/image_raw"
run bash -lc "source /opt/ros/humble/setup.bash; source ~/follow_ws/install/setup.bash 2>/dev/null || true; timeout 5s ros2 topic echo /camera/depth/camera_info --once"

section "Done"
echo "If no depth topics are present, start the Orbbec launch in another terminal and rerun this script."
