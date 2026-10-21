#!/usr/bin/env bash
# dataset_generator (the HDF5 recorder, src/dataset_generator): h5py. Not a
# driver, always built, so install.sh and the Dockerfile run it unconditionally;
# console's rosdep step also installs it (package.xml: exec_depend python3-h5py).
#
# Usage:
#   sudo bash dataset.sh       install whatever is missing
#   bash dataset.sh --check    no root, installs nothing: exit 0 if everything listed
#                          here is in place, else exit 1 and list what is missing
#                          on stderr. Works on plain ROS 2, without console.

PACKAGES=(python3-h5py)

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
