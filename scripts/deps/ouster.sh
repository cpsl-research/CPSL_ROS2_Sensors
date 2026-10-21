#!/usr/bin/env bash
# Ouster OS-series lidar: build dependencies of ouster-ros, plus fping for
# scripts/find_ouster_ip.sh. Selecting cyclonedds as the RMW is shell setup and
# stays in install.sh.
#
# Usage:
#   sudo bash ouster.sh       install whatever is missing
#   bash ouster.sh --check    no root, installs nothing: exit 0 if everything listed
#                          here is in place, else exit 1 and list what is missing
#                          on stderr. Works on plain ROS 2, without console.

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

PACKAGES=(
    fping
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-tf2-eigen"
    "ros-${ROS_DISTRO}-rviz2"
    build-essential
    libeigen3-dev
    libjsoncpp-dev
    libspdlog-dev
    libcurl4-openssl-dev
    libpcap-dev
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure "${PACKAGES[@]}"
