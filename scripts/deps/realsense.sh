#!/usr/bin/env bash
# Intel RealSense: the ROS wrapper packages from apt, then librealsense built
# from source with the user-space RSUSB backend (no kernel patches, works in a
# container) and the graphical tools (realsense-viewer). This is the
# configuration verified in Docker (planning Track H, H2/H6).
#
# The source and build tree stay in /opt/librealsense, so re-running after the
# installed files are lost only repeats `make install`, not the compile.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

SRC_DIR=/opt/librealsense
INSTALLED=/usr/local/lib/librealsense2.so

apt_ensure \
    git ca-certificates cmake make build-essential pkg-config \
    ros-jazzy-librealsense2 \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-camera-msgs \
    ros-jazzy-realsense2-description \
    libusb-1.0-0-dev libudev-dev libssl-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

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
