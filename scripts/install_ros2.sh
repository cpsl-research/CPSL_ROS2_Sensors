#!/usr/bin/env bash
# Installs ROS2 Jazzy on Ubuntu 24.04 (Noble), then hands off to install.sh.
# Usage: bash scripts/install_ros2.sh [--sensors <list>] [options]
# Must be run from the CPSL_ROS2_Sensors workspace root.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== ROS2 Jazzy Installation ==="

# Locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

echo "=== Installing ROS2 Jazzy Desktop ==="
sudo apt install -y ros-jazzy-desktop python3-rosdep

echo "=== Initialising rosdep ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "=== Handing off to install.sh ==="
exec bash "${SCRIPT_DIR}/install.sh" "$@"
