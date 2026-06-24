#!/usr/bin/env bash
# Rebuilds the CPSL_ROS2_Sensors workspace packages.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_ROOT"

# Detect if running in Docker container
if [ -f /.dockerenv ]; then
    set +u
    . /opt/ros/jazzy/setup.sh
    set -u
    if command -v poetry &>/dev/null; then
        eval "$(poetry env activate)"
    fi
else
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
    fi
fi

BUILD_BASE="build"
INSTALL_BASE="install"
LOG_BASE="log"
IGNORED_PACKAGES=()

if [ -f /.dockerenv ]; then
    BUILD_BASE="build_docker"
    INSTALL_BASE="install_docker"
    LOG_BASE="log_docker"
    
    if [ -f /etc/cpsl_sensors_list ]; then
        SENSORS_LIST=$(cat /etc/cpsl_sensors_list)
        if [ "$SENSORS_LIST" = "all" ]; then SENSORS_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi
        
        if ! echo ",$SENSORS_LIST," | grep -q ",radar,"; then
            IGNORED_PACKAGES+=(ti_radar_connect raw_radar_msgs)
        fi
        if ! echo ",$SENSORS_LIST," | grep -q ",livox,"; then
            IGNORED_PACKAGES+=(livox_ros_driver2)
        fi
        if ! echo ",$SENSORS_LIST," | grep -q ",ouster,"; then
            IGNORED_PACKAGES+=(ouster_ros ouster_sensor_msgs)
        fi
        if ! echo ",$SENSORS_LIST," | grep -q ",leapmotion,"; then
            IGNORED_PACKAGES+=(leap_msgs leap_node ohrc_leap)
        fi
        if ! echo ",$SENSORS_LIST," | grep -q ",vicon,"; then
            IGNORED_PACKAGES+=(vicon_bridge)
        fi
    fi
fi

echo "=== Running Colcon Build ==="
if [ ${#IGNORED_PACKAGES[@]} -gt 0 ]; then
    colcon --log-base "$LOG_BASE" build \
        --base-paths src \
        --build-base "$BUILD_BASE" \
        --install-base "$INSTALL_BASE" \
        --symlink-install \
        --packages-ignore "${IGNORED_PACKAGES[@]}"
else
    colcon --log-base "$LOG_BASE" build \
        --base-paths src \
        --build-base "$BUILD_BASE" \
        --install-base "$INSTALL_BASE" \
        --symlink-install
fi

echo "Build complete. Remember to source your workspace setup file (e.g. source ${INSTALL_BASE}/setup.bash)"
