# Shared helpers for scripts/deps/<driver>.sh. Sourced, not executed.
#
# The contract every deps script keeps:
#   - system dependencies only (apt packages, SDKs built under /opt and
#     installed to /usr/local). Nothing in the workspace, the user's shell rc,
#     group membership or sensor IP configuration — install.sh owns those.
#   - run as root, no sudo inside: the Dockerfile and a container run it
#     directly, install.sh runs it with `sudo bash`.
#   - idempotent and quick when already satisfied, so a container can re-run
#     it on every start.
#   - a non-zero exit means the driver's dependencies are NOT in place.

set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: $(basename "$0") must run as root (e.g. sudo bash $0)." >&2
    exit 1
fi

_APT_UPDATED=false

# apt_ensure pkg... — installs whichever of the named packages are missing.
# Skips apt-get update entirely when everything is already installed.
apt_ensure() {
    local missing=()
    local pkg
    for pkg in "$@"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ${#missing[@]} -eq 0 ]; then
        echo "apt: already installed: $*"
        return 0
    fi
    if [ "$_APT_UPDATED" = false ]; then
        apt-get update
        _APT_UPDATED=true
    fi
    echo "apt: installing ${missing[*]}"
    apt-get install -y --no-install-recommends "${missing[@]}"
}
