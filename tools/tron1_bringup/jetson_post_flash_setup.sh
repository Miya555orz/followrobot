#!/usr/bin/env bash
# Jetson post-flash setup for the FCR/TRON1 migration.
# Run this on Jetson after JetPack/Ubuntu first boot is complete.

set -euo pipefail

section() {
  printf '\n===== %s =====\n' "$1"
}

section "System summary"
hostname
uname -a
lsb_release -a || true
cat /etc/nv_tegra_release || true
df -h
free -h

section "APT base tools"
sudo apt update
sudo apt install -y \
  curl \
  gnupg \
  lsb-release \
  software-properties-common \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  can-utils \
  usbutils \
  v4l-utils \
  net-tools \
  ripgrep

section "ROS 2 Humble apt source"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

section "ROS 2 Humble packages"
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-rqt-robot-steering \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-camera-info-manager \
  ros-humble-diagnostic-updater \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher

section "rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

section "Shell setup"
if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
  echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
fi

section "NVIDIA / JetPack checks"
cat /etc/nv_tegra_release || true
apt list --installed 2>/dev/null | grep nvidia-jetpack || true
command -v nvcc >/dev/null 2>&1 && nvcc --version || true

section "Next steps"
cat <<'EOF'
If this script completed, run:

  mkdir -p ~/follow_ws/src
  cd ~/follow_ws/src
  git clone -b main https://github.com/Miya555orz/followrobot.git fcr_ros2_3
  cd ~/follow_ws
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
    --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg

Then run:

  cd ~/follow_ws/src/fcr_ros2_3
  bash tools/tron1_bringup/jetson_rs2_preflight.sh
EOF
