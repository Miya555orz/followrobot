#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$HOME/followrobot}"
mkdir -p "$ROOT/upstream/cuiangA" "$ROOT/upstream/limxdynamics"

clone_or_update() {
  local url="$1"
  local dir="$2"
  if [ -d "$dir/.git" ]; then
    git -C "$dir" fetch --all --prune
  else
    git clone "$url" "$dir"
  fi
}

clone_or_update https://github.com/cuiangA/fcr_ros2_3.git "$ROOT/upstream/cuiangA/fcr_ros2_3"
clone_or_update https://github.com/limxdynamics/limxsdk-lowlevel.git "$ROOT/upstream/limxdynamics/limxsdk-lowlevel"
clone_or_update https://github.com/limxdynamics/tron1-rl-deploy-ros2.git "$ROOT/upstream/limxdynamics/tron1-rl-deploy-ros2"
clone_or_update https://github.com/limxdynamics/tron1-gazebo-ros2.git "$ROOT/upstream/limxdynamics/tron1-gazebo-ros2"
clone_or_update https://github.com/limxdynamics/tron1-robot-description.git "$ROOT/upstream/limxdynamics/tron1-robot-description"
clone_or_update https://github.com/limxdynamics/robot-visualization.git "$ROOT/upstream/limxdynamics/robot-visualization"
clone_or_update https://github.com/limxdynamics/ros2-bridger.git "$ROOT/upstream/limxdynamics/ros2-bridger"
clone_or_update https://github.com/limxdynamics/robot-joystick.git "$ROOT/upstream/limxdynamics/robot-joystick"

echo "Upstream repositories are present under $ROOT/upstream."
