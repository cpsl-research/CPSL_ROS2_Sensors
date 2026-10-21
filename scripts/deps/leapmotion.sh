#!/usr/bin/env bash
# Leap Motion: the Ultraleap hand-tracking SDK from Ultraleap's apt repo.
# The python bindings and their cffi build go into the workspace venv, not the
# system, so they are done by `scripts/setup_venv.sh --leapmotion` after this.
#
# repo.ultraleap.com is not always reachable; if it is not, this fails rather
# than pretending the SDK is installed. Callers that want to carry on without
# Leap Motion (the Dockerfile does) handle the non-zero exit themselves.
#
# Usage:
#   sudo bash leapmotion.sh       install whatever is missing
#   bash leapmotion.sh --check    no root, installs nothing: exit 0 if the packages
#                                 below are installed, else exit 1 and list what is
#                                 missing on stderr.
#
# --check does NOT verify the Ultraleap apt source (key and list file) beyond
# the ultraleap-hand-tracking package itself being installed.

KEYRING=/usr/share/keyrings/ultraleap-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/ultraleap.list

PREREQ=(curl ca-certificates gnupg)
PACKAGES=("${PREREQ[@]}" ultraleap-hand-tracking)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

# The SDK package comes from the Ultraleap repo, so it is installed last.
apt_ensure "${PREREQ[@]}"

if [ ! -f "$LIST" ]; then
    tmp=$(mktemp)
    if ! curl -fsSL --connect-timeout 5 https://repo.ultraleap.com/apt/public.key -o "$tmp"; then
        rm -f "$tmp"
        echo "ERROR: repo.ultraleap.com is unreachable; Ultraleap SDK not installed." >&2
        exit 1
    fi
    gpg --dearmor --yes -o "$KEYRING" < "$tmp"
    rm -f "$tmp"
    echo "deb [signed-by=$KEYRING] https://repo.ultraleap.com/apt stable main" > "$LIST"
    _APT_UPDATED=false    # the new repo needs an update before install
fi

apt_ensure ultraleap-hand-tracking
