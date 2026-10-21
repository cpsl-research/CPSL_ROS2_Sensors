#!/usr/bin/env bash
# Intel RealSense: the ROS wrapper packages from apt, then librealsense built
# from source with the user-space RSUSB backend (no kernel patches, works in a
# container) and the graphical tools (realsense-viewer). This is the
# configuration verified in Docker (planning Track H, H2/H6).
#
# The source and build tree stay in /opt/librealsense, so re-running after the
# installed files are lost only repeats `make install`, not the compile.
#
# Usage:
#   sudo bash realsense.sh       install whatever is missing
#   bash realsense.sh --check    no root, installs nothing: exit 0 if the apt packages
#                                below and the installed library are present, else
#                                exit 1 and list what is missing on stderr.
#
# --check does NOT verify: a librealsense built and installed somewhere other
# than $INSTALLED (override the path with REALSENSE_INSTALLED), or that it was
# built with the RSUSB backend.
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

SRC_DIR=/opt/librealsense
INSTALLED="${REALSENSE_INSTALLED:-/usr/local/lib/librealsense2.so}"

PACKAGES=(
    git ca-certificates cmake make build-essential pkg-config
    "ros-${ROS_DISTRO}-librealsense2"
    "ros-${ROS_DISTRO}-realsense2-camera"
    "ros-${ROS_DISTRO}-realsense2-camera-msgs"
    "ros-${ROS_DISTRO}-realsense2-description"
    libusb-1.0-0-dev libudev-dev libssl-dev
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ! -e "$INSTALLED" ]; then
        missing+=("librealsense")
    fi
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure "${PACKAGES[@]}"

if [ -e "$INSTALLED" ]; then
    echo "realsense: librealsense already installed ($INSTALLED)."
    exit 0
fi

if [ ! -d "$SRC_DIR/.git" ]; then
    git clone --depth 1 https://github.com/IntelRealSense/librealsense.git "$SRC_DIR"
fi
mkdir -p "$SRC_DIR/build"
cd "$SRC_DIR/build"
if [ ! -f Makefile ]; then
    cmake .. -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release \
        -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
fi
make -j"$(nproc)"
make install
ldconfig
echo "realsense: librealsense installed."
