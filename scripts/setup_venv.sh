#!/usr/bin/env bash
# Creates (or updates) the workspace's python venv with uv.
#
# Usage:
#   bash scripts/setup_venv.sh [--leapmotion]
#
# The venv is created with --system-site-packages: the ROS nodes and colcon
# itself import rclpy and friends from /opt/ros, which is not pip-installable,
# so the venv has to see the system python's packages as well as its own.
#
# Location: $UV_PROJECT_ENVIRONMENT if set, otherwise <workspace>/.venv. Set it
# whenever the same source tree is shared between a host and a container (a
# bind mount) so the two never share, and break, one venv.
#
# --leapmotion additionally installs the Leap Motion python bindings from the
# leapc-python-bindings submodule and builds their cffi module. They are not in
# pyproject.toml because that submodule is normally not checked out and uv
# cannot lock a path dependency that is not on disk; the sync below is
# --inexact so a later run without --leapmotion does not uninstall them.
# Requires the Ultraleap SDK (scripts/deps/leapmotion.sh) to be installed.
#
# Activate it before building, so colcon's python nodes get this interpreter:
#   source "${UV_PROJECT_ENVIRONMENT:-.venv}/bin/activate"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

LEAPMOTION=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --leapmotion) LEAPMOTION=true; shift ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

if ! command -v uv &>/dev/null; then
    echo "ERROR: uv not found. Install it (https://docs.astral.sh/uv/) and re-run."
    exit 1
fi

PYTHON=/usr/bin/python3.12
VENV="${UV_PROJECT_ENVIRONMENT:-$WORKSPACE_ROOT/.venv}"
export UV_PROJECT_ENVIRONMENT="$VENV"

# A venv that exists but cannot see the system site-packages (e.g. one made by
# a bare `uv sync`) would build fine and then fail at runtime importing rclpy,
# so it is recreated rather than reused.
if [ -f "$VENV/pyvenv.cfg" ] && grep -q '^include-system-site-packages = true' "$VENV/pyvenv.cfg"; then
    echo "venv: reusing $VENV"
else
    echo "venv: creating $VENV (system site-packages visible)"
    uv venv --clear --system-site-packages --python "$PYTHON" "$VENV"
fi

uv sync --frozen --inexact

if [[ "$LEAPMOTION" == true ]]; then
    BINDINGS="submodules/leapc-python-bindings"
    if [ ! -d "$BINDINGS/leapc-python-api" ]; then
        echo "ERROR: $BINDINGS is not checked out (git submodule update --init $BINDINGS)."
        exit 1
    fi
    uv pip install --python "$VENV/bin/python" -e "$BINDINGS/leapc-python-api"
    (cd "$BINDINGS" && "$VENV/bin/python" build_cffi.py)
fi

echo "venv ready: $VENV"
