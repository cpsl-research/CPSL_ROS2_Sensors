#!/usr/bin/env bash
# TI radar: no system dependencies beyond what rosdep installs for
# CPSL_TI_Radar_ROS2 (run `rosdep install --from-paths src ...` after checking
# the submodule out). This script exists so callers never special-case a driver.
# Serial access (dialout group) is host user setup and lives in install.sh.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

echo "radar: no system dependencies outside rosdep."
