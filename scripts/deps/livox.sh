#!/usr/bin/env bash
# Livox Mid360: Livox-SDK2, built from source with gcc-9 into /usr/local.
# The source and build tree stay in /opt/Livox-SDK2, so re-running after the
# installed files are lost (e.g. a recreated container that kept /opt) only
# repeats `make install`, not the compile.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

SDK_DIR=/opt/Livox-SDK2
INSTALLED=/usr/local/lib/liblivox_lidar_sdk_shared.so

apt_ensure git ca-certificates gcc-9 g++-9 cmake make

if [ -f "$INSTALLED" ]; then
    echo "livox: Livox-SDK2 already installed ($INSTALLED)."
    exit 0
fi

if [ ! -d "$SDK_DIR/.git" ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git "$SDK_DIR"
fi
mkdir -p "$SDK_DIR/build"
cd "$SDK_DIR/build"
if [ ! -f Makefile ]; then
    cmake .. -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9
fi
make -j"$(nproc)"
make install
ldconfig
echo "livox: Livox-SDK2 installed."
