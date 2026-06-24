#!/usr/bin/env bash
# Automated check for docker build, workspace mounting, and container compilation.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== Phase 1: Docker Build Verification ==="
bash "$WORKSPACE_ROOT/docker/install_cpsl_sensors_docker.sh" --sensors radar --tag test_tag:v1

echo "=== Phase 2: Volume Mount Verification ==="
SENTINEL_FILE="$WORKSPACE_ROOT/docker_mount_test_sentinel.tmp"
echo "docker-mount-test" > "$SENTINEL_FILE"

# Clean up sentinel file on script exit
trap 'rm -f "$SENTINEL_FILE"' EXIT

# Run check inside container
if docker run --rm -v "$WORKSPACE_ROOT:/workspace/CPSL_ROS2_Sensors" test_tag:v1 \
    cat /workspace/CPSL_ROS2_Sensors/docker_mount_test_sentinel.tmp | grep -q "docker-mount-test"; then
    echo "SUCCESS: Host workspace is successfully mounted."
else
    echo "ERROR: Shared volume mount verification failed."
    exit 1
fi

echo "=== Phase 3: Compilation Verification ==="
docker run --rm -v "$WORKSPACE_ROOT:/workspace/CPSL_ROS2_Sensors" test_tag:v1 \
    bash /workspace/CPSL_ROS2_Sensors/scripts/rebuild.sh

echo "=== ALL CHECKS PASSED ==="
