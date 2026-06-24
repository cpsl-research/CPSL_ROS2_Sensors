#!/usr/bin/env bash
# Scans link-local IP range to find active IP addresses of the lidar sensor.
set -euo pipefail

START_IP="${1:-169.254.0.1}"
END_IP="${2:-169.254.255.255}"

if ! command -v fping &>/dev/null; then
    echo "ERROR: fping is not installed. Installing..."
    sudo apt-get update && sudo apt-get install -y fping
fi

echo "Scanning IP range: $START_IP to $END_IP"
fping -a -r 1 -g "$START_IP" "$END_IP" 2>/dev/null || true
