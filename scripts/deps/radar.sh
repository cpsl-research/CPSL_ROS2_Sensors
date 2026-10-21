#!/usr/bin/env bash
# TI radar: no system dependencies beyond what rosdep installs for
# CPSL_TI_Radar (ros2/ packages; run `rosdep install --from-paths src ...` after checking
# the submodule out). This script exists so callers never special-case a driver.
# Serial access (dialout group) is host user setup and lives in install.sh.
#
# Usage:
#   sudo bash radar.sh       install whatever is missing
#   bash radar.sh --check    no root, installs nothing: exit 0 if everything listed
#                          here is in place, else exit 1 and list what is missing
#                          on stderr. Works on plain ROS 2, without console.
#                          (Nothing to check here: always exits 0.)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

echo "radar: no system dependencies outside rosdep."
