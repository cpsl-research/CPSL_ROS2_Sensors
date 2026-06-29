#!/usr/bin/env bash
# Host udev setup script for Intel RealSense cameras.
# Must be run as root (or with sudo).

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (e.g. using sudo)." 1>&2
   exit 1
fi

RULES_FILE="/etc/udev/rules.d/99-realsense-libusb.rules"
URL="https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules"

echo "Setting up Intel RealSense udev rules..."

# Try to download the latest rules from Intel's GitHub repository
DOWNLOAD_SUCCESS=false
if command -v curl &>/dev/null; then
    echo "Attempting to download rules via curl..."
    if curl -sSL --connect-timeout 5 -o "$RULES_FILE" "$URL"; then
        DOWNLOAD_SUCCESS=true
    fi
elif command -v wget &>/dev/null; then
    echo "Attempting to download rules via wget..."
    if wget -q --timeout=5 -O "$RULES_FILE" "$URL"; then
        DOWNLOAD_SUCCESS=true
    fi
fi

if [ "$DOWNLOAD_SUCCESS" = true ]; then
    echo "Successfully downloaded official Intel RealSense rules."
    # Ensure IMU/HID rules are appended to guarantee they exist
    cat << 'EOF' >> "$RULES_FILE"

# Intel RealSense IMU (HID / IIO) custom rules
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="8086", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="iio", ATTRS{idVendor}=="8086", MODE:="0666", GROUP:="plugdev"
EOF
else
    echo "Download failed or tools unavailable. Writing offline fallback rules..."
    cat << 'EOF' > "$RULES_FILE"
# Fallback device rules for Intel RealSense devices (R200, F200, SR300 LR200, ZR300, D400, L500, T200)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a80", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0a66", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa3", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa2", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aa5", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0abf", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0acb", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad0", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad6", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af6", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b37", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b61", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE:="0666", GROUP:="plugdev"

# RealSense IMU (HID / IIO) custom rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="04b4", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="8086", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="iio", ATTRS{idVendor}=="8086", MODE:="0666", GROUP:="plugdev"
EOF
    echo "Fallback rules written."
fi

# Make sure plugdev group exists, if not create it
if ! getent group plugdev >/dev/null; then
    echo "Creating plugdev group..."
    groupadd plugdev || true
fi

echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "Intel RealSense udev rules setup completed successfully."
