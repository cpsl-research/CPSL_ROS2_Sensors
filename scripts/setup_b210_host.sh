#!/usr/bin/env bash
# Host-only setup for a USRP B210 (a container cannot do these: they are kernel/host
# settings). Run once per machine, after scripts/deps/b210.sh:
#     sudo bash scripts/setup_b210_host.sh
# Every step prints what it will do first and is idempotent. Re-login afterwards
# (limits apply at login), then replug the B210 into a SuperSpeed (USB 3) port.
#
# Steps:
#   1. udev: uhd-host already ships /usr/lib/udev/rules.d/60-uhd-host.rules
#      (MODE 0666); just reload and trigger.
#   2. limits: /etc/security/limits.d/90-b210.conf, memlock unlimited + rtprio 99 for
#      the invoking user's primary group (uhd-host's own file only covers group usrp).
#   3. USB autosuspend off: runtime now, and persistent via
#      /etc/modprobe.d/usbcore-autosuspend.conf (GRUB is not touched).
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: run as root: sudo bash $0" >&2
    exit 1
fi
USER_NAME="${SUDO_USER:-}"
if [ -z "$USER_NAME" ] || [ "$USER_NAME" = root ]; then
    echo "ERROR: run via sudo from your own user (SUDO_USER is empty/root)." >&2
    exit 1
fi
GROUP_NAME="$(id -gn "$USER_NAME")"

write_if_differs() {  # write_if_differs path content
    if [ -f "$1" ] && [ "$(cat "$1")" = "$2" ]; then
        echo "  unchanged: $1"
    else
        printf '%s\n' "$2" > "$1"
        echo "  wrote: $1"
    fi
}

# 1. udev
RULE=/usr/lib/udev/rules.d/60-uhd-host.rules
echo "[1/3] udev: reload rules and trigger (rule file from uhd-host: $RULE)"
if [ ! -f "$RULE" ]; then
    echo "ERROR: $RULE missing; run scripts/deps/b210.sh first." >&2
    exit 1
fi
udevadm control --reload
udevadm trigger

# 2. limits
LIMITS=/etc/security/limits.d/90-b210.conf
LIMITS_BODY="# B210 / sar_b210: unlimited locked memory and realtime priority for @$GROUP_NAME
@$GROUP_NAME - memlock unlimited
@$GROUP_NAME - rtprio 99"
echo "[2/3] limits: writing $LIMITS with:"
echo "$LIMITS_BODY" | sed 's/^/    /'
write_if_differs "$LIMITS" "$LIMITS_BODY"

# 3. USB autosuspend
PARAM=/sys/module/usbcore/parameters/autosuspend
MODPROBE=/etc/modprobe.d/usbcore-autosuspend.conf
echo "[3/3] USB autosuspend: set $PARAM = -1 now, and persist in $MODPROBE:"
echo "    options usbcore autosuspend=-1"
echo -1 > "$PARAM"
write_if_differs "$MODPROBE" "options usbcore autosuspend=-1"
echo "  (usbcore is built into most kernels; if the modprobe.d file has no effect after"
echo "   reboot, check $PARAM and add usbcore.autosuspend=-1 to the kernel command line.)"

echo
echo "Done. Now: log out and back in, then replug the B210 into a USB 3 port."
