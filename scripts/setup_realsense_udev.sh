#!/usr/bin/env bash
# Shim: the RealSense host udev setup is now tools/setup/realsense_host_setup.py
# (vendored, pinned v2.58.4 rules; backup of a differing file; scoped udev trigger).
# Run this as yourself, not with sudo: the helper calls sudo per command.
set -euo pipefail

HELPER="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/tools/setup/realsense_host_setup.py"

if [[ $EUID -eq 0 ]]; then
    echo "Do not run this as root. Run, as your own user:" 1>&2
    echo "  python3 $HELPER --apply" 1>&2
    exit 2
fi

exec python3 "$HELPER" --apply "$@"
