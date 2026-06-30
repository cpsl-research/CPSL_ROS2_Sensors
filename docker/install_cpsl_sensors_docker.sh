#!/usr/bin/env bash
# Wrapper script to build and configure the CPSL_ROS2_Sensors Docker environment.
# Can be run from any directory.

set -euo pipefail

# Determine script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── helper function ────────────────────────────────────────────────────────────
header() { echo; echo "=== $* ==="; }

# ── parse arguments ────────────────────────────────────────────────────────────
SENSORS="all"
LIVOX_IP=""
OUSTER_HOSTNAME=""
SKIP_BUILD=false
GPU=false
HOST_PARENT_INTERFACE="eth0"
TAG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sensors)          SENSORS="$2";         shift 2 ;;
        --livox-ip)         LIVOX_IP="$2";        shift 2 ;;
        --ouster-hostname)  OUSTER_HOSTNAME="$2"; shift 2 ;;
        --parent-interface) HOST_PARENT_INTERFACE="$2"; shift 2 ;;
        --skip-build)       SKIP_BUILD=true;      shift ;;
        --gpu)              GPU=true;             shift ;;
        -t|--tag)           TAG="$2";             shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done


# ── check docker installation ──────────────────────────────────────────────────
header "Checking Docker Engine"
if ! command -v docker &>/dev/null; then
    echo "ERROR: Docker command not found. Please install Docker Engine first."
    echo "Refer to: https://docs.docker.com/engine/install/"
    exit 1
fi

if [[ "$SKIP_BUILD" == false ]]; then
    if ! docker info &>/dev/null; then
        echo "ERROR: Docker daemon is not running, or current user does not have permission."
        echo "Please start Docker or add your user to the 'docker' group: sudo usermod -aG docker \$USER"
        exit 1
    fi
    echo "Docker is installed and running."
else
    echo "Docker check skipped (skip-build requested)."
fi

# Validate SENSORS
VALID_SENSORS="radar livox ouster realsense leapmotion vicon all"
for s in $(echo "$SENSORS" | tr ',' ' '); do
    if ! echo "$VALID_SENSORS" | grep -qw "$s"; then
        echo "ERROR: unknown sensor '$s'. Valid options: radar, livox, ouster, realsense, leapmotion, vicon, all"
        exit 1
    fi
done

# If SENSORS is "all", expand it
IF_SENSORS="$SENSORS"
if [[ "$SENSORS" == "all" ]]; then
    IF_SENSORS="radar,livox,ouster,realsense,leapmotion,vicon"
fi

# ── generate .env file ─────────────────────────────────────────────────────────
header "Generating .env Configuration File"
ENV_FILE="$SCRIPT_DIR/.env"
ROOT_ENV_FILE="$(cd "$SCRIPT_DIR/.." && pwd)/.env"

# Extract display configurations
DISPLAY_VAL="${DISPLAY:-:0}"

# Base IPs (derived or user provided)
HOST_LIVOX_IP=""
if [[ -n "$LIVOX_IP" ]]; then
    HOST_LIVOX_IP="192.168.1.${LIVOX_IP}"
else
    HOST_LIVOX_IP="192.168.1.78" # Default Livox Host IP
fi

HOST_OUSTER_IP=""
if [[ -n "$OUSTER_HOSTNAME" ]]; then
    HOST_OUSTER_IP="$OUSTER_HOSTNAME"
else
    HOST_OUSTER_IP="169.254.1.1" # Default Ouster Host IP
fi

HOST_RADAR_IP="192.168.33.30" # Default Radar Host IP

update_env_file() {
    local target_file="$1"
    if [[ -f "$target_file" ]]; then
        echo "Updating existing $target_file in-place (non-destructive)..."
        local temp_env
        temp_env=$(mktemp)
        local -A updated_keys
        
        while IFS= read -r line || [[ -n "$line" ]]; do
            if [[ -z "$line" || "$line" =~ ^# ]]; then
                echo "$line" >> "$temp_env"
                continue
            fi
            
            if [[ "$line" =~ ^([^=]+)=(.*)$ ]]; then
                local key="${BASH_REMATCH[1]}"
                key=$(echo "$key" | xargs)
                
                case "$key" in
                    SENSORS)
                        echo "SENSORS=$IF_SENSORS" >> "$temp_env"
                        updated_keys[SENSORS]=1
                        ;;
                    HOST_LIVOX_IP)
                        echo "HOST_LIVOX_IP=$HOST_LIVOX_IP" >> "$temp_env"
                        updated_keys[HOST_LIVOX_IP]=1
                        ;;
                    HOST_OUSTER_IP)
                        echo "HOST_OUSTER_IP=$HOST_OUSTER_IP" >> "$temp_env"
                        updated_keys[HOST_OUSTER_IP]=1
                        ;;
                    HOST_RADAR_IP)
                        echo "HOST_RADAR_IP=$HOST_RADAR_IP" >> "$temp_env"
                        updated_keys[HOST_RADAR_IP]=1
                        ;;
                    DISPLAY)
                        echo "DISPLAY=$DISPLAY_VAL" >> "$temp_env"
                        updated_keys[DISPLAY]=1
                        ;;
                    HOST_PARENT_INTERFACE)
                        echo "HOST_PARENT_INTERFACE=$HOST_PARENT_INTERFACE" >> "$temp_env"
                        updated_keys[HOST_PARENT_INTERFACE]=1
                        ;;
                    *)
                        echo "$line" >> "$temp_env"
                        ;;
                esac
            else
                echo "$line" >> "$temp_env"
            fi
        done < "$target_file"
        
        [[ -z "${updated_keys[SENSORS]:-}" ]] && echo "SENSORS=$IF_SENSORS" >> "$temp_env"
        [[ -z "${updated_keys[HOST_LIVOX_IP]:-}" ]] && echo "HOST_LIVOX_IP=$HOST_LIVOX_IP" >> "$temp_env"
        [[ -z "${updated_keys[HOST_OUSTER_IP]:-}" ]] && echo "HOST_OUSTER_IP=$HOST_OUSTER_IP" >> "$temp_env"
        [[ -z "${updated_keys[HOST_RADAR_IP]:-}" ]] && echo "HOST_RADAR_IP=$HOST_RADAR_IP" >> "$temp_env"
        [[ -z "${updated_keys[DISPLAY]:-}" ]] && echo "DISPLAY=$DISPLAY_VAL" >> "$temp_env"
        [[ -z "${updated_keys[HOST_PARENT_INTERFACE]:-}" ]] && echo "HOST_PARENT_INTERFACE=$HOST_PARENT_INTERFACE" >> "$temp_env"
        
        mv "$temp_env" "$target_file"
    else
        echo "Creating new $target_file..."
        cat <<EOF > "$target_file"
# Auto-generated by install_cpsl_sensors_docker.sh
SENSORS=$IF_SENSORS
HOST_LIVOX_IP=$HOST_LIVOX_IP
HOST_OUSTER_IP=$HOST_OUSTER_IP
HOST_RADAR_IP=$HOST_RADAR_IP
DISPLAY=$DISPLAY_VAL
HOST_PARENT_INTERFACE=$HOST_PARENT_INTERFACE
EOF
    fi
}

update_env_file "$ENV_FILE"
update_env_file "$ROOT_ENV_FILE"

echo "Written configuration to $ENV_FILE:"
cat "$ENV_FILE"

# ── build docker image ─────────────────────────────────────────────────────────
if [[ "$SKIP_BUILD" == false ]]; then
    # Clone/update submodules on the host before building the container
    header "Updating Git Submodules on Host"
    WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
    
    contains() { echo ",$1," | grep -q ",$2,"; }

    # Navigate to workspace root for git operations
    pushd "$WORKSPACE_ROOT" > /dev/null
    
    if contains "$IF_SENSORS" "radar"; then
        echo "Updating radar submodule..."
        git submodule update --init --recursive src/CPSL_TI_Radar_ROS2 || echo "Warning: git submodule update failed (not a git repository?)"
    fi
    if contains "$IF_SENSORS" "livox"; then
        echo "Updating livox submodule..."
        git submodule update --init --recursive src/CPSL_ROS_livox_ros_driver2 || echo "Warning: git submodule update failed (not a git repository?)"
    fi
    if contains "$IF_SENSORS" "ouster"; then
        echo "Updating ouster submodule..."
        git submodule update --init --recursive src/ouster-ros || echo "Warning: git submodule update failed (not a git repository?)"
    fi
    if contains "$IF_SENSORS" "vicon"; then
        echo "Updating vicon submodule..."
        git submodule update --init --recursive src/ros2-vicon-bridge || echo "Warning: git submodule update failed (not a git repository?)"
    fi
    if contains "$IF_SENSORS" "leapmotion"; then
        echo "Updating leapmotion submodules..."
        git submodule update --init --recursive src/CPSL_ROS2_LeapMotion || echo "Warning: git submodule update failed (not a git repository?)"
        git submodule update --init --recursive submodules/leapc-python-bindings || echo "Warning: git submodule update failed (not a git repository?)"
    fi
    
    popd > /dev/null

    header "Building Docker Image"
    if [[ "$GPU" == true ]]; then
        BASE_IMG="nvidia/cuda:13.3.0-cudnn-devel-ubuntu24.04"
        DEFAULT_TAG="cpsl_sensors:gpu"
    else
        BASE_IMG="ubuntu:24.04"
        DEFAULT_TAG="cpsl_sensors:cpu"
    fi

    TARGET_TAG="${TAG:-$DEFAULT_TAG}"

    echo "Target tag: $TARGET_TAG"
    echo "Base image: $BASE_IMG"
    echo "Sensors   : $IF_SENSORS"

    docker build \
        --build-arg BASE_IMAGE="$BASE_IMG" \
        --build-arg SENSORS="$IF_SENSORS" \
        -t "$TARGET_TAG" \
        -f "$SCRIPT_DIR/Dockerfile" \
        "$WORKSPACE_ROOT"
else
    header "Building Docker Image (SKIPPED)"
fi

echo "Done."

