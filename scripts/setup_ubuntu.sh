#!/usr/bin/env bash
set -euo pipefail

echo "This script installs only conservative base tools."
echo "It does not install ROS 2, Gazebo, TRON1 dependencies, or hardware rules automatically."
echo "Read docs/linux_setup.md first."

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  curl \
  wget \
  gnupg \
  lsb-release \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions

echo "Base tools installed."
echo "Next: install ROS 2 Humble from the official ROS documentation, then run the validation commands in docs/linux_setup.md."
