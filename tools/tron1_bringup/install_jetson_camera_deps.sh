#!/usr/bin/env bash
set -euo pipefail

# Install Jetson-side dependencies required for the RS2 + Orbbec/Sony camera
# bring-up path. This script installs system packages only; build/install/log
# are intentionally not version-controlled and should be regenerated locally.
#
# Optional:
#   APT_PROXY=http://127.0.0.1:7892 bash tools/tron1_bringup/install_jetson_camera_deps.sh

if [[ ${EUID} -ne 0 ]]; then
  exec sudo --preserve-env=APT_PROXY "$0" "$@"
fi

declare -a apt_opts=()
if [[ -n "${APT_PROXY:-}" ]]; then
  apt_opts+=(
    -o "Acquire::http::Proxy=${APT_PROXY}"
    -o "Acquire::https::Proxy=${APT_PROXY}"
  )
  echo "Using apt proxy: ${APT_PROXY}"
fi

apt update "${apt_opts[@]}"

apt install -y "${apt_opts[@]}" \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-backward-ros \
  ros-humble-image-publisher \
  ros-humble-diagnostic-updater \
  ros-humble-tf2-sensor-msgs \
  ros-humble-camera-calibration \
  ros-humble-rclcpp-components \
  libopencv \
  libopencv-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  nlohmann-json3-dev \
  libssl-dev \
  libgl1-mesa-dev \
  usbutils \
  v4l-utils

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
orbbec_rules="${repo_root}/src/orbbec_camera/scripts/99-obsensor-libusb.rules"

if [[ -f "${orbbec_rules}" ]]; then
  rm -f /etc/udev/rules.d/99-obsensor-ros1-libusb.rules
  install -m 0644 "${orbbec_rules}" /etc/udev/rules.d/99-obsensor-libusb.rules
  udevadm control --reload-rules
  udevadm trigger
  echo "Installed Orbbec udev rule: /etc/udev/rules.d/99-obsensor-libusb.rules"
else
  echo "WARN: Orbbec udev rule not found at ${orbbec_rules}" >&2
fi

cat <<'EOF'

Dependency installation complete.

Recommended low-parallel build on Jetson:

  cd ~/follow_ws
  source /opt/ros/humble/setup.bash
  MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
    --packages-up-to orbbec_camera sony_camera_pkg

If the Orbbec camera was already plugged in before installing udev rules,
unplug and replug the camera once before launching the driver.
EOF
