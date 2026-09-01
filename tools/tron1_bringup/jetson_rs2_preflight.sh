#!/usr/bin/env bash
# Read-only bring-up diagnostics for Jetson Orin Nano + DJI RS2 CAN.
# Run this on the Jetson terminal, then paste the output into Codex.

set -u

section() {
  printf '\n===== %s =====\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

section "Machine"
run hostname
run whoami
run uname -a
run lsb_release -a

section "CPU / Memory / Disk"
run nproc
run free -h
run df -h

section "Network"
run ip -brief address
run ip route

section "ROS 2"
run bash -lc "source /opt/ros/humble/setup.bash >/dev/null 2>&1; which ros2; ros2 --version; printenv | grep -E 'ROS|AMENT|COLCON' || true"

section "USB / Serial / CAN devices"
run lsusb
run bash -lc "ls -l /dev/ttyUSB* /dev/ttyACM* /dev/can* 2>/dev/null || true"
run ip -details link show

section "Kernel messages, recent device events"
run dmesg --ctime --level=err,warn
run bash -lc "dmesg --ctime | tail -120"

section "Optional tools"
run bash -lc "command -v candump || true"
run bash -lc "command -v cansend || true"
run bash -lc "command -v ros2 || true"

section "FCR workspace if present"
run bash -lc "test -d ~/follow_ws && cd ~/follow_ws && pwd && colcon list 2>/dev/null | head -80 || true"

section "Done"
printf 'Paste everything from this script output back to Codex.\n'
