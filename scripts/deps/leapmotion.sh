#!/usr/bin/env bash
# Leap Motion: the Ultraleap hand-tracking SDK from Ultraleap's apt repo.
# The python bindings and their cffi build go into the workspace venv, not the
# system, so they are done by `scripts/setup_venv.sh --leapmotion` after this.
#
# repo.ultraleap.com is not always reachable; if it is not, this fails rather
# than pretending the SDK is installed. Callers that want to carry on without
# Leap Motion (the Dockerfile does) handle the non-zero exit themselves.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

KEYRING=/usr/share/keyrings/ultraleap-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/ultraleap.list

apt_ensure curl ca-certificates gnupg

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
