#!/usr/bin/env bash
# Livox Mid360: Livox-SDK2, built from source with gcc-9 into /usr/local.
# The source and build tree stay in /opt/Livox-SDK2, so re-running after the
# installed files are lost (e.g. a recreated container that kept /opt) only
# repeats `make install`, not the compile.
#
# Usage:
#   sudo bash livox.sh       install whatever is missing
#   bash livox.sh --check    no root, installs nothing: exit 0 if the apt packages
#                            below and the installed SDK library are present, else
#                            exit 1 and list what is missing on stderr.
#
# --check does NOT verify: the cap_net_raw capability on `ping` (set below only
# when needed), or an SDK built and installed somewhere other than $INSTALLED
# (override the path with LIVOX_SDK_INSTALLED).

SDK_DIR=/opt/Livox-SDK2
INSTALLED="${LIVOX_SDK_INSTALLED:-/usr/local/lib/liblivox_lidar_sdk_shared.so}"

# iproute2 and iputils-ping are the tools the Livox host helper
# (tools/setup/livox_host_setup.py) shells out to: `ip` (which interfaces to
# search) and `ping` (reachability check / sweep). Without them, running the
# helper from a container finds nothing.
PACKAGES=(git ca-certificates gcc-9 g++-9 cmake make iproute2 iputils-ping)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ! -e "$INSTALLED" ]; then
        missing+=("Livox-SDK2")
    fi
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure "${PACKAGES[@]}"

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
