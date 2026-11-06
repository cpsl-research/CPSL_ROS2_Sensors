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
#   3. USB autosuspend off for the B210: udev rule /etc/udev/rules.d/61-b210-power.rules
#      sets power/control=on on the device (persistent, per device; modprobe.d does not
#      work where usbcore is built into the kernel), plus the global runtime
#      parameter now. An earlier version of this script wrote
#      /etc/modprobe.d/usbcore-autosuspend.conf; it is removed if unchanged.
#      GRUB is not touched.
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
POWER_RULE=/etc/udev/rules.d/61-b210-power.rules
POWER_BODY='# B210: no USB runtime autosuspend (power/control=on); 2500:0020 unloaded, 0021/0022 after firmware
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2500", ATTR{idProduct}=="002[0-2]", TEST=="power/control", ATTR{power/control}="on"'
echo "[3/3] USB autosuspend: set $PARAM = -1 now (runtime only), and persist per device in $POWER_RULE:"
echo "$POWER_BODY" | sed 's/^/    /'
echo -1 > "$PARAM"
write_if_differs "$POWER_RULE" "$POWER_BODY"

# Stale file from an earlier version of this script (no effect with built-in usbcore).
OLD=/etc/modprobe.d/usbcore-autosuspend.conf
if [ -f "$OLD" ]; then
    echo "  stale $OLD found, content:"
    sed 's/^/    /' "$OLD"
    if [ "$(cat "$OLD")" = "options usbcore autosuspend=-1" ]; then
        rm -f "$OLD"
        echo "  removed: $OLD (content was exactly what this script wrote)"
    else
        echo "  left in place: content differs from what this script wrote"
    fi
fi
udevadm control --reload
udevadm trigger --subsystem-match=usb --action=add
for d in /sys/bus/usb/devices/*; do
    [ "$(cat "$d/idVendor" 2>/dev/null)" = 2500 ] && echo "  $(basename "$d") power/control = $(cat "$d/power/control" 2>/dev/null)"
done

echo
echo "Done. Now: log out and back in, then replug the B210 into a USB 3 port."
