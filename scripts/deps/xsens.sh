#!/usr/bin/env bash
# Xsens MTi IMU/AHRS (xsens_mti_ros2_driver, submodule src/xsens_mti_ros2_driver):
# apt build libraries only. The bundled xspublic library is built by the driver's
# own CMake (ExternalProject, plain make), so there is no SDK step here. Serial
# access (the `dialout` group) is host user setup and lives in install.sh.
# nmea_msgs and mavros_msgs are hard find_package() deps even for an AHRS.
#
# Usage:
#   sudo bash xsens.sh       install whatever is missing
#   bash xsens.sh --check    no root, installs nothing: exit 0 if everything listed
#                          here is in place, else exit 1 and list what is missing
#                          on stderr. Works on plain ROS 2, without console.

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

PACKAGES=(
    "ros-${ROS_DISTRO}-nmea-msgs"
    "ros-${ROS_DISTRO}-mavros-msgs"
    libeigen3-dev
    build-essential
    make
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
