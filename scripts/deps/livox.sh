#!/usr/bin/env bash
# Livox Mid360: Livox-SDK2, built from source with gcc-9 into /usr/local.
# The source and build tree stay in /opt/Livox-SDK2, so re-running after the
# installed files are lost (e.g. a recreated container that kept /opt) only
# repeats `make install`, not the compile.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

SDK_DIR=/opt/Livox-SDK2
INSTALLED=/usr/local/lib/liblivox_lidar_sdk_shared.so

apt_ensure git ca-certificates gcc-9 g++-9 cmake make

# Tools the Livox host helper (tools/setup/livox_host_setup.py) shells out to:
# `ip` (which interfaces to search) and `ping` (reachability check / sweep).
# Without them, running the helper from a container finds nothing.
apt_ensure iproute2 iputils-ping

# ping needs raw sockets. Where unprivileged ICMP is off
# (net.ipv4.ping_group_range), a non-root user can use it only if the binary
# carries cap_net_raw or is setuid. The iputils-ping postinst sets the
# capability only when setcap is present, so set it here if it is missing.
PING_BIN="$(command -v ping || true)"
if [ -n "$PING_BIN" ] && [ ! -u "$PING_BIN" ]; then
    apt_ensure libcap2-bin
    if ! getcap "$PING_BIN" 2>/dev/null | grep -q cap_net_raw; then
        setcap cap_net_raw+ep "$PING_BIN" \
            || echo "livox: WARNING could not set cap_net_raw on $PING_BIN" >&2
    fi
fi

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
