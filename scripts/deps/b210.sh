#!/usr/bin/env bash
# Ettus USRP B210: a shim over sar_b210's host doctor (src/sar_b210/tools/setup/host_setup.py),
# which owns the real list: UHD apt packages, B2xx images, udev rules, limits, autosuspend.
#
# Usage:
#   sudo bash b210.sh          deps only (apt + B2xx images; needs network; also the container)
#   sudo bash b210.sh --host   host-only steps for a new machine: udev power rule, memlock/
#                              rtprio limits, USB autosuspend (the container cannot do these)
#   bash b210.sh --check       no root, installs nothing, opens no radio: exit 0 if deps are in
#                              place and any attached B210 has power/control == on, else exit 1
#                              and list what is missing on stderr.
# --check does NOT verify: that a B210 is attached, its USB speed, or image/UHD versions.

HOST_SETUP="$(dirname "${BASH_SOURCE[0]}")/../../src/sar_b210/tools/setup/host_setup.py"
if [ ! -f "$HOST_SETUP" ]; then
    echo "ERROR: src/sar_b210 is not checked out (b210 deps live there)." >&2
    echo "  Run: git submodule update --init src/sar_b210  (or enable B210 on console's Sensors page)" >&2
    exit 1
fi

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    python3 "$HOST_SETUP" --scope deps --check || exit 1
    exec python3 "$HOST_SETUP" --only autosuspend --check
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

if [ "${1:-}" = "--host" ]; then
    exec python3 "$HOST_SETUP" --scope host --apply --yes
fi
exec python3 "$HOST_SETUP" --scope deps --apply --yes
