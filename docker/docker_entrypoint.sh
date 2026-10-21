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

# Dynamically create IIO device nodes (RealSense IMU) inside the container
# (This bypasses the OCI runtime/runc colon-parsing limitation on /dev/iio:device*)
if [ -d "/sys/bus/iio/devices" ]; then
    for dev_path in /sys/bus/iio/devices/iio:device*; do
        if [ -d "$dev_path" ]; then
            dev_name=$(basename "$dev_path")
            if [ -f "$dev_path/dev" ]; then
                # Read major:minor (e.g. 507:0)
                dev_info=$(cat "$dev_path/dev")
                major=$(echo "$dev_info" | cut -d':' -f1)
                minor=$(echo "$dev_info" | cut -d':' -f2)
                
                # Create character node inside the container if it doesn't exist
                if [ ! -c "/dev/$dev_name" ]; then
                    mknod "/dev/$dev_name" c "$major" "$minor"
                    chmod 666 "/dev/$dev_name"
                    chown root:plugdev "/dev/$dev_name" || true
                fi
            fi
        fi
    done
fi

exec "$@"
