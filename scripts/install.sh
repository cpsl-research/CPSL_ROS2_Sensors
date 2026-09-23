#!/usr/bin/env bash
# One-shot install script for CPSL_ROS2_Sensors.
# Must be run from the workspace root (the directory containing src/ and pyproject.toml).
#
# Usage:
#   bash scripts/install.sh [--sensors <list>] [--livox-ip <XX>] \
#                           [--ouster-hostname <host>] [--skip-build]
#
# --sensors   Comma-separated subset of: radar,livox,ouster,realsense,leapmotion,vicon,usb_cam
#             Default: radar,livox
# --livox-ip  Last two digits of the Livox Mid360 serial number (sets static host IP)
# --ouster-hostname  Hostname or IP of the Ouster sensor
# --skip-build  Skip the colcon build step

set -euo pipefail

# ── helpers ────────────────────────────────────────────────────────────────────
header() { echo; echo "=== $* ==="; }
contains() { echo ",$1," | grep -q ",$2,"; }

# ── argument parsing ───────────────────────────────────────────────────────────
SENSORS="radar,livox"
LIVOX_IP=""
OUSTER_HOSTNAME=""
SKIP_BUILD=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sensors)        SENSORS="$2";        shift 2 ;;
        --livox-ip)       LIVOX_IP="$2";       shift 2 ;;
        --ouster-hostname) OUSTER_HOSTNAME="$2"; shift 2 ;;
        --skip-build)     SKIP_BUILD=true;     shift ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

VALID_SENSORS="radar livox ouster realsense leapmotion vicon usb_cam"
for s in $(echo "$SENSORS" | tr ',' ' '); do
    if ! echo "$VALID_SENSORS" | grep -qw "$s"; then
        echo "ERROR: unknown sensor '$s'. Valid options: $VALID_SENSORS"
        exit 1
    fi
done

SHELL_EXT="$(basename "$SHELL")"
RC_FILE="${HOME}/.${SHELL_EXT}rc"
WORKSPACE_ROOT="$(pwd)"

header "Step 1: Summary"
echo "Workspace root : $WORKSPACE_ROOT"
echo "Sensors        : $SENSORS"
echo "Shell RC file  : $RC_FILE"
[[ -n "$LIVOX_IP" ]]        && echo "Livox host IP  : 192.168.1.${LIVOX_IP}"
[[ -n "$OUSTER_HOSTNAME" ]] && echo "Ouster hostname: $OUSTER_HOSTNAME"
[[ "$SKIP_BUILD" == true ]] && echo "Colcon build   : SKIPPED"

# ── submodule init/deinit ─────────────────────────────────────────────────────
header "Step 2: Submodule setup"

# Parallel arrays: submodule path → sensor name that requires it
SUBMODULE_PATHS=(
    "src/CPSL_TI_Radar_ROS2"
    "src/CPSL_ROS_livox_ros_driver2"
    "src/ouster-ros"
    "src/ros2-vicon-bridge"
    "src/CPSL_ROS2_LeapMotion"
    "submodules/leapc-python-bindings"
)
SUBMODULE_SENSORS=(
    "radar"
    "livox"
    "ouster"
    "vicon"
    "leapmotion"
    "leapmotion"
)

for i in "${!SUBMODULE_PATHS[@]}"; do
    path="${SUBMODULE_PATHS[$i]}"
    sensor="${SUBMODULE_SENSORS[$i]}"
    if contains "$SENSORS" "$sensor"; then
        echo "  [init]   $path ($sensor)"
        git submodule init "$path"
        git submodule update "$path"
    else
        # Only deinit if the directory is currently populated
        if [ -n "$(ls -A "$path" 2>/dev/null)" ]; then
            echo "  [deinit] $path ($sensor not selected)"
            git submodule deinit -f "$path"
        else
            echo "  [skip]   $path ($sensor not selected, already empty)"
        fi
    fi
done

# ── uv + venv ──────────────────────────────────────────────────────────────────
header "Step 3: uv"
if ! command -v uv &>/dev/null; then
    echo "uv not found — installing..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="${HOME}/.local/bin:$PATH"
fi
echo "uv version: $(uv --version)"

header "Step 4: Per-sensor system dependencies"
# One script per driver (scripts/deps/), shared with the Dockerfile and with
# console. Each runs as root, installs system packages/SDKs only, and is a no-op
# when already satisfied.
for s in $(echo "$SENSORS" | tr ',' ' '); do
    echo "--- deps: $s ---"
    sudo bash scripts/deps/"$s".sh
done

header "Step 5: Python environment"
# .venv with system site-packages, so the ROS python packages stay visible.
if contains "$SENSORS" leapmotion; then
    bash scripts/setup_venv.sh --leapmotion
else
    bash scripts/setup_venv.sh
fi

header "Step 6: rosdep"
sudo apt-get update
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --rosdistro=jazzy --skip-keys "raw_radar_msgs"

# ── per-sensor setup ───────────────────────────────────────────────────────────
header "Step 7: Sensor-specific setup"

# radar ────────────────────────────────────────────────────────────────────────
if contains "$SENSORS" radar; then
    echo "--- radar ---"
    sudo usermod -a -G dialout "$USER"
    echo ""
    echo "  ACTION REQUIRED after install:"
    echo "  1. Log out and back in so the dialout group takes effect."
    echo "  2. For DCA1000 ethernet capture, run once:"
    echo "       sudo sysctl -w net.core.rmem_max=134217728"
    echo "     To make it permanent:"
    echo "       echo 'net.core.rmem_max=134217728' | sudo tee /etc/sysctl.d/99-radar.conf"
    echo "       sudo sysctl --system"
fi

# livox ────────────────────────────────────────────────────────────────────────
if contains "$SENSORS" livox; then
    echo "--- livox ---"

    # Livox-SDK2 itself was installed by scripts/deps/livox.sh in Step 4.

    # Configure Livox driver package.xml
    bash src/CPSL_ROS_livox_ros_driver2/build_CPSL_ROS2_Sensors.sh jazzy

    # Optionally patch MID360_config.json with the user's host IP
    if [[ -n "$LIVOX_IP" ]]; then
        CONFIG_FILE="src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json"
        SENSOR_LAST=$(python3 -c "
import json, sys
with open('$CONFIG_FILE') as f:
    d = json.load(f)
ip = d['lidar_summary_info']['lidar_net_info'][0]['ip']
print(ip.split('.')[-1])
")
        HOST_IP="192.168.1.${LIVOX_IP}"
        SENSOR_IP="192.168.1.${SENSOR_LAST}"
        python3 - <<PYEOF
import json
with open('$CONFIG_FILE') as f:
    d = json.load(f)
d['lidar_summary_info']['lidar_net_info'][0]['ip'] = '$SENSOR_IP'
d['host_net_info']['host_ip'] = '$HOST_IP'
with open('$CONFIG_FILE', 'w') as f:
    json.dump(d, f, indent=4)
print("Patched $CONFIG_FILE: host_ip=$HOST_IP")
PYEOF
    fi
fi

# ouster ───────────────────────────────────────────────────────────────────────
if contains "$SENSORS" ouster; then
    echo "--- ouster ---"
    # Build dependencies were installed by scripts/deps/ouster.sh in Step 4.
    if ! grep -q "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" "$RC_FILE" 2>/dev/null; then
        echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$RC_FILE"
        echo "  Appended RMW_IMPLEMENTATION to $RC_FILE"
    fi

    if [[ -n "$OUSTER_HOSTNAME" ]]; then
        PARAMS_FILE="src/cpsl_ros2_sensors_bringup/ouster_configs/driver_params.yaml"
        python3 - <<PYEOF
import re
with open('$PARAMS_FILE') as f:
    content = f.read()
content = re.sub(r"(sensor_hostname:\s*')[^']*(')", r"\g<1>$OUSTER_HOSTNAME\2", content)
with open('$PARAMS_FILE', 'w') as f:
    f.write(content)
print("Patched $PARAMS_FILE: sensor_hostname=$OUSTER_HOSTNAME")
PYEOF
    fi
fi

# usb_cam ──────────────────────────────────────────────────────────────────────
if contains "$SENSORS" usb_cam; then
    echo "--- usb_cam ---"
    sudo usermod -a -G video "$USER"
    echo "  Log out and back in so the video group takes effect."
fi

# realsense, leapmotion, vicon: system dependencies only, installed in Step 4
# (the Leap Motion python bindings in Step 5). Nothing host-specific to add.

# ── colcon build ───────────────────────────────────────────────────────────────
if [[ "$SKIP_BUILD" == false ]]; then
    header "Step 8: Colcon build"
    source "${UV_PROJECT_ENVIRONMENT:-$WORKSPACE_ROOT/.venv}/bin/activate"
    source /opt/ros/jazzy/setup.bash

    if contains "$SENSORS" radar; then
        python -m colcon build \
            --packages-select raw_radar_msgs \
            --symlink-install
    fi

    if contains "$SENSORS" ouster; then
        python -m colcon build \
            --packages-select ouster_ros ouster_sensor_msgs \
            --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
    fi

    python -m colcon build \
        --base-paths src \
        --symlink-install
else
    header "Step 8: Colcon build (SKIPPED)"
fi

# ── source workspace ───────────────────────────────────────────────────────────
header "Step 9: Sourcing workspace"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.${SHELL_EXT}"
if [ -f "$SETUP_FILE" ]; then
    # shellcheck disable=SC1090
    source "$SETUP_FILE"
    echo "Sourced $SETUP_FILE"
else
    echo "Note: $SETUP_FILE not found — source it manually after the build completes."
fi

# ── post-install summary ───────────────────────────────────────────────────────
header "Step 10: Post-install summary"
echo "Installed sensors: $SENSORS"
echo ""
echo "Manual hardware configuration still required:"

if contains "$SENSORS" radar; then
    echo ""
    echo "  TI Radar (serial):"
    echo "    - Log out/in for dialout group to take effect"
    echo "    - Connect radars: ttyACM0=front, ttyACM1=back (or ttyUSB0 for IWR6843)"
    echo "    - For DCA1000: set PC ethernet adapter to 192.168.33.30/24 (single radar)"
    echo "      or 192.168.1.57/24 (multi-radar / co-located with Livox)"
    echo "    - Apply rmem_max tuning (see commands printed above)"
fi

if contains "$SENSORS" livox; then
    echo ""
    echo "  Livox Mid360:"
    if [[ -z "$LIVOX_IP" ]]; then
        echo "    - Set PC ethernet adapter to 192.168.1.XX/24 (XX = last 2 digits of serial)"
        echo "    - Update host_ip in src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json"
    else
        echo "    - PC host IP already set to 192.168.1.${LIVOX_IP} in MID360_config.json"
        echo "    - Set PC ethernet adapter to 192.168.1.${LIVOX_IP}/24"
    fi
fi

if contains "$SENSORS" ouster; then
    echo ""
    echo "  Ouster:"
    if [[ -z "$OUSTER_HOSTNAME" ]]; then
        echo "    - Set PC ethernet adapter to 169.254.1.1/255.255.0.0 (link-local)"
        echo "    - Update sensor_hostname in src/cpsl_ros2_sensors_bringup/ouster_configs/driver_params.yaml"
    else
        echo "    - sensor_hostname already set to $OUSTER_HOSTNAME in driver_params.yaml"
        echo "    - Set PC ethernet adapter to 169.254.1.1/255.255.0.0 (link-local)"
    fi
    echo "    - Log out/in (or source $RC_FILE) for RMW_IMPLEMENTATION to take effect"
fi

echo ""
echo "Setup complete."
