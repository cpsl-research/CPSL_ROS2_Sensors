#!/usr/bin/env bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source the workspace setup if it has been built
if [ -f "/workspace/CPSL_ROS2_Sensors/install_docker/setup.bash" ]; then
    source "/workspace/CPSL_ROS2_Sensors/install_docker/setup.bash"
elif [ -f "/workspace/CPSL_ROS2_Sensors/install/setup.bash" ]; then
    source "/workspace/CPSL_ROS2_Sensors/install/setup.bash"
fi

exec "$@"
