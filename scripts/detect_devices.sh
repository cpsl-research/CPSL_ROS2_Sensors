#!/usr/bin/env bash
# Standalone wrapper script to run host device detection and generate env configuration.
# Can be run from any directory.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Running Host Device Discovery ==="
python3 "$SCRIPT_DIR/detect_devices.py"
echo "=== Host Device Discovery Completed ==="
