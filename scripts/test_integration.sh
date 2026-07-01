#!/usr/bin/env bash
# Automated integration check for workspace compilation, device detection,
# and hardware launch testing.
# Can be run from the repository root.

set -euo pipefail

# ── Helpers ───────────────────────────────────────────────────────────────────
header() { echo -e "\n=== $* ==="; }
info() { echo "INFO: $*"; }
success() { echo -e "\e[32mSUCCESS: $*\e[0m"; }
error() { echo -e "\e[31mERROR: $*\e[0m"; }

shutdown_launch() {
    local pid="$1"
    info "Shutting down bringup process (PID: $pid)..."
    kill -2 "$pid" 2>/dev/null || true
    
    # Wait up to 5 seconds for clean exit
    local count=0
    while [ $count -lt 5 ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    
    # Force kill if still running
    info "Bringup process did not exit cleanly. Force terminating..."
    kill -9 "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

# ── Default Configuration ─────────────────────────────────────────────────────
TIMEOUT=20
FORCE_HOST=false
FORCE_DOCKER=false
SENSORS="all"

# ── Helpers ───────────────────────────────────────────────────────────────────
contains() { echo ",$1," | grep -q ",$2,"; }

# ── Argument Parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)       FORCE_HOST=true; shift ;;
        --docker)     FORCE_DOCKER=true; shift ;;
        --timeout)    TIMEOUT="$2"; shift 2 ;;
        --sensors)    SENSORS="$2"; shift 2 ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if [[ "$SENSORS" == "all" ]]; then
    SENSORS="radar,livox,ouster,realsense,leapmotion,vicon"
fi

CLI_SENSORS="$SENSORS"

if [[ "$FORCE_HOST" == true && "$FORCE_DOCKER" == true ]]; then
    error "Cannot force both --host and --docker. Select one."
    exit 1
fi

# ── Determine Execution Environment ───────────────────────────────────────────
IS_DOCKER=false
if [ -f "/.dockerenv" ] || grep -q 'docker' /proc/1/cgroup 2>/dev/null; then
    IS_DOCKER=true
fi

if [[ "$FORCE_HOST" == true ]]; then
    IS_DOCKER=false
elif [[ "$FORCE_DOCKER" == true ]]; then
    IS_DOCKER=true
fi

header "Phase 1: Environment & Dependency Scan"
info "Execution platform: $( [[ "$IS_DOCKER" == true ]] && echo "Docker Container" || echo "Host Machine" )"
info "Verification timeout: ${TIMEOUT}s"

# Check directories
if [ ! -d "src" ] || [ ! -d "docker" ]; then
    error "Must run test script from the repository root directory containing src/ and docker/."
    exit 1
fi

# Verify dependencies based on mode
if [[ "$IS_DOCKER" == false ]]; then
    # Check docker command
    if command -v docker &>/dev/null; then
        success "Docker is installed."
    else
        info "Docker not detected on host. (Skipping container build tests)."
    fi

    # Check local ROS2
    if ! command -v ros2 &>/dev/null; then
        info "ros2 command not in PATH. Attempting to source /opt/ros/jazzy/setup.bash..."
        if [ -f "/opt/ros/jazzy/setup.bash" ]; then
            set +u
            # shellcheck disable=SC1091
            source /opt/ros/jazzy/setup.bash || true
            set -u
        fi
    fi

    if command -v ros2 &>/dev/null; then
        set +u
        distro="${ROS_DISTRO:-unknown}"
        set -u
        success "ROS2 environment detected: $distro"
    else
        error "ROS2 Jazzy is not sourced or installed on the host. Sourcing '/opt/ros/jazzy/setup.bash' failed."
        exit 1
    fi
else
    # Inside docker: ROS2 is always installed
    set +u
    distro="${ROS_DISTRO:-unknown}"
    set -u
    success "Running inside Docker. ROS2: $distro"
fi

header "Phase 2: Compilation & Workspace Build Verification"
BUILD_FAILED=false

if [[ "$IS_DOCKER" == true ]]; then
    info "Running clean workspace build inside container..."
    if colcon --log-base log_docker build \
              --build-base build_docker \
              --install-base install_docker \
              --packages-select cpsl_ros2_sensors_bringup platform_descriptions; then
        success "Container workspace built successfully."
        # Sourcing container workspace
        set +u
        # shellcheck disable=SC1090
        source install_docker/setup.bash
        set -u
    else
        error "Container workspace build failed!"
        BUILD_FAILED=true
    fi
else
    info "Running workspace build on host..."
    if colcon build --symlink-install --packages-select cpsl_ros2_sensors_bringup platform_descriptions; then
        success "Host workspace built successfully."
        # Sourcing host workspace
        set +u
        # shellcheck disable=SC1090
        source install/setup.bash
        set -u
    else
        error "Host workspace build failed!"
        BUILD_FAILED=true
    fi
fi

if [[ "$BUILD_FAILED" == true ]]; then
    exit 1
fi

header "Phase 3: Hardware Scan"
# Run scanner to ensure we have the latest environment setup
info "Scanning for active sensor interfaces..."
if [[ "$IS_DOCKER" == false ]]; then
    python3 scripts/detect_devices.py > /dev/null || true
fi

# Load device env mappings
ENV_FILE=$( [[ "$IS_DOCKER" == true ]] && echo "docker/.env" || echo ".env" )
if [ -f "$ENV_FILE" ]; then
    # Sourcing env safely by stripping comments
    set +u
    while IFS= read -r line || [ -n "$line" ]; do
        # Ignore comments and empty lines
        if [[ ! "$line" =~ ^# ]] && [[ -n "$line" ]]; then
            export "$line" || true
        fi
    done < "$ENV_FILE"
    set -u
    # Restore the CLI SENSORS selection so it's not overridden by the env file
    SENSORS="$CLI_SENSORS"
else
    info "No configuration file found at $ENV_FILE. Skipping hardware checks."
fi

# Check active mappings and requested sensor tests
REALSENSE_ACTIVE=false
RADAR_ACTIVE=false

if contains "$SENSORS" "realsense"; then
    if [[ -n "${REALSENSE_DEV_0:-}" && "${REALSENSE_DEV_0:-}" != "/dev/null" ]]; then
        REALSENSE_ACTIVE=true
        success "RealSense camera detected on host port mapping: $REALSENSE_DEV_0"
    fi
fi

if contains "$SENSORS" "radar"; then
    if [[ -n "${FRONT_RADAR_CLI:-}" && "${FRONT_RADAR_CLI:-}" != "/dev/null" ]]; then
        RADAR_ACTIVE=true
        success "TI Radar detected on host port mapping: $FRONT_RADAR_CLI"
    fi
fi

header "Phase 4: Dynamic ROS2 Smoke Tests"

# RealSense Test
REALSENSE_TEST="SKIPPED"
if [[ "$REALSENSE_ACTIVE" == true ]]; then
    info "Starting RealSense ROS2 Smoke Test (Timeout: ${TIMEOUT}s)..."
    
    # Run bringup launch in the background
    ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py \
        realsense_enable:=true \
        rviz:=false > realsense_test_run.log 2>&1 &
    LAUNCH_PID=$!
    
    # Wait for sensor initialization and verify topic publishing
    TOPIC_VERIFIED=false
    ELAPSED=0
    
    # Polling loop
    while [ $ELAPSED -lt "$TIMEOUT" ]; do
        sleep 2
        ELAPSED=$((ELAPSED + 2))
        
        # Check topic lists
        MATCHED_TOPIC=$(ros2 topic list 2>/dev/null | grep "/camera/camera/color/image_raw" | head -n 1 || true)
        if [[ -n "$MATCHED_TOPIC" ]]; then
            info "RealSense topic '$MATCHED_TOPIC' detected in list after ${ELAPSED}s. Verifying active data stream..."
            # Try to echo once with timeout to check for active frames on the matched topic
            if timeout 5 ros2 topic echo --once "$MATCHED_TOPIC" &>/dev/null; then
                TOPIC_VERIFIED=true
                break
            fi
        fi
    done
    
    # Clean shutdown
    shutdown_launch "$LAUNCH_PID"
    
    if [[ "$TOPIC_VERIFIED" == true ]]; then
        success "RealSense Camera ROS2 nodes successfully published image frames!"
        REALSENSE_TEST="PASSED"
    else
        error "RealSense Camera ROS2 nodes failed to stream images within timeout."
        REALSENSE_TEST="FAILED"
    fi
fi

# TI Radar Test
RADAR_TEST="SKIPPED"
if [[ "$RADAR_ACTIVE" == true ]]; then
    info "Starting TI Radar ROS2 Smoke Test (Timeout: ${TIMEOUT}s)..."
    
    # Select appropriate config file (container paths vs host paths)
    CONFIG_FILE="front_radar_IWR1843_stress_test.json"
    if [[ "$IS_DOCKER" == true ]]; then
        CONFIG_FILE="front_radar_IWR1843_stress_test_docker.json"
    fi

    # Run bringup launch in the background
    ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py \
        front_radar_enable:=true \
        front_radar_config_file:="$CONFIG_FILE" \
        rviz:=false > radar_test_run.log 2>&1 &
    LAUNCH_PID=$!
    
    # Wait and verify if node starts up
    NODE_VERIFIED=false
    ELAPSED=0
    
    while [ $ELAPSED -lt "$TIMEOUT" ]; do
        sleep 2
        ELAPSED=$((ELAPSED + 2))
        
        if ros2 node list 2>/dev/null | grep -q "ti_radar_connect"; then
            NODE_VERIFIED=true
            break
        fi
    done
    
    # Clean shutdown
    shutdown_launch "$LAUNCH_PID"
    
    if [[ "$NODE_VERIFIED" == true ]]; then
        success "TI Radar connection node successfully initialized!"
        RADAR_TEST="PASSED"
    else
        error "TI Radar connection node failed to start within timeout."
        RADAR_TEST="FAILED"
    fi
fi

header "Phase 5: GUI / RViz2 Driver Smoke Test"
GUI_TEST="SKIPPED"
if [[ -n "${DISPLAY:-}" ]]; then
    info "Starting RViz2 GUI Smoke Test (Timeout: 5s)..."
    
    # Run rviz2 in the background using timeout
    set +e
    timeout 5 rviz2 > rviz2_test_run.log 2>&1
    RV_STATUS=$?
    set -e
    
    # If the exit status is 124, it means the process timed out, which implies
    # it started and ran for 5 seconds without crashing!
    if [ $RV_STATUS -eq 124 ]; then
        # Double check log for Mesa or iris driver failures
        if grep -qE "MESA: error|failed to load driver|failed to create dri3 screen" rviz2_test_run.log; then
            error "RViz2 launched but encountered MESA driver/DRI3 errors:"
            grep -E "MESA: error|failed to load driver|failed to create dri3 screen" rviz2_test_run.log || true
            GUI_TEST="FAILED"
        else
            success "RViz2 launched and initialized hardware rendering successfully."
            GUI_TEST="PASSED"
        fi
    else
        error "RViz2 failed to launch (Exit code: $RV_STATUS). Log output:"
        cat rviz2_test_run.log
        GUI_TEST="FAILED"
    fi
else
    info "DISPLAY environment variable not set. Skipping GUI smoke test."
fi

# ── Summary Report ────────────────────────────────────────────────────────────
header "Test Verification Summary"
echo "-----------------------------------------------"
echo -e "Compilation Test         : \e[32mPASSED\e[0m"
echo -e "Intel RealSense Camera   : $( [[ "$REALSENSE_TEST" == "PASSED" ]] && echo -e "\e[32mPASSED\e[0m" || ( [[ "$REALSENSE_TEST" == "FAILED" ]] && echo -e "\e[31mFAILED\e[0m" || echo -e "\e[33mSKIPPED (Not connected)\e[0m" ) )"
echo -e "TI Radar Sensor          : $( [[ "$RADAR_TEST" == "PASSED" ]] && echo -e "\e[32mPASSED\e[0m" || ( [[ "$RADAR_TEST" == "FAILED" ]] && echo -e "\e[31mFAILED\e[0m" || echo -e "\e[33mSKIPPED (Not connected)\e[0m" ) )"
echo -e "GUI / RViz2 Rendering    : $( [[ "$GUI_TEST" == "PASSED" ]] && echo -e "\e[32mPASSED\e[0m" || ( [[ "$GUI_TEST" == "FAILED" ]] && echo -e "\e[31mFAILED\e[0m" || echo -e "\e[33mSKIPPED (No Display)\e[0m" ) )"
echo "-----------------------------------------------"

if [[ "$REALSENSE_TEST" == "FAILED" || "$RADAR_TEST" == "FAILED" || "$GUI_TEST" == "FAILED" ]]; then
    error "Some integration checks failed. Refer to logs (*_test_run.log)."
    exit 1
else
    success "All integration checks passed successfully!"
fi
