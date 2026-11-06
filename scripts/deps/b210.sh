#!/usr/bin/env bash
# Ettus USRP B210: UHD from Ubuntu apt (4.6 on noble) plus the B2xx FPGA/firmware
# images in UHD's default images directory. Host-only steps (udev reload, memlock,
# rtprio, USB autosuspend) are NOT here: see scripts/setup_b210_host.sh.
#
# Usage:
#   sudo bash b210.sh          install whatever is missing (needs network: apt, and
#                              files.ettus.com for the images; fails loudly without)
#   bash b210.sh --check       no root, installs nothing, opens no radio: exit 0 if the
#                              apt packages and the B210 FPGA image + B200 firmware are
#                              present, else exit 1 and list what is missing on stderr.
#
# --check also requires power/control == on for an attached Ettus (2500) USB device.
# --check does NOT verify: that a B210 is attached, its USB speed, or the image
# versions matching the installed UHD.
IMAGES_DIR=/usr/share/uhd/images
IMAGE_FILES=(usrp_b210_fpga.bin usrp_b200_fw.hex)

PACKAGES=(
    libuhd-dev uhd-host libyaml-cpp-dev libboost-system-dev cmake ninja-build
)

# Handled before _common.sh, whose root check would fail for a normal user.
if [ "${1:-}" = "--check" ]; then
    missing=()
    for pkg in "${PACKAGES[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        fi
    done
    for f in "${IMAGE_FILES[@]}"; do
        if [ ! -e "$IMAGES_DIR/$f" ]; then
            missing+=("image:$IMAGES_DIR/$f")
        fi
    done
    # Autosuspend: per-device (udev rule from setup_b210_host.sh), only when a B210 is attached.
    for d in /sys/bus/usb/devices/*; do
        [ "$(cat "$d/idVendor" 2>/dev/null)" = 2500 ] || continue
        pc="$(cat "$d/power/control" 2>/dev/null)"
        [ "$pc" = on ] || missing+=("usb-power:$(basename "$d") power/control=$pc (want on; run setup_b210_host.sh, replug)")
    done
    if [ ${#missing[@]} -ne 0 ]; then
        echo "missing: ${missing[*]}" >&2
        exit 1
    fi
    exit 0
fi

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

# The container's apt lists are empty; apt_ensure updates before installing.
apt_ensure "${PACKAGES[@]}"

have_images=true
for f in "${IMAGE_FILES[@]}"; do
    [ -e "$IMAGES_DIR/$f" ] || have_images=false
done
if [ "$have_images" = true ]; then
    echo "b210: images already in $IMAGES_DIR."
    exit 0
fi

echo "b210: downloading B2xx images into $IMAGES_DIR (needs network)."
if ! uhd_images_downloader -t b2xx -i "$IMAGES_DIR"; then
    echo "ERROR: uhd_images_downloader failed (no network to files.ettus.com?)." >&2
    exit 1
fi
for f in "${IMAGE_FILES[@]}"; do
    if [ ! -e "$IMAGES_DIR/$f" ]; then
        echo "ERROR: $IMAGES_DIR/$f still missing after download." >&2
        exit 1
    fi
done
echo "b210: UHD and B2xx images installed."
