#!/usr/bin/env bash
# Host udev setup script for TI Radar boards.
# Generates role-based symlinks (e.g. /dev/ti_front_radar_cli) dynamically 
# by reading mapping configurations from docker/device_config.json.
# Must be run as root (or with sudo).

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (e.g. using sudo)." 1>&2
   exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="$SCRIPT_DIR/../docker/device_config.json"
RULES_FILE="/etc/udev/rules.d/98-ti-radar.rules"

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "Error: device_config.json not found at $CONFIG_FILE" 1>&2
    exit 1
fi

echo "Generating TI Radar udev rules from $CONFIG_FILE..."

python3 -c '
import json
import sys
import os

config_path = sys.argv[1]
rules_path = sys.argv[2]

with open(config_path, "r") as f:
    config = json.load(f)

rules = [
    "# udev rules for Texas Instruments Evaluation Modules (Radars)",
    "# Auto-generated from device_config.json. DO NOT EDIT DIRECTLY.",
    "",
    "# General permissions fallback rules for TI Radar bridges:",
    "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0451\", ATTRS{idProduct}==\"bef3\", MODE=\"0666\", GROUP=\"dialout\"",
    "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea70\", MODE=\"0666\", GROUP=\"dialout\"",
    ""
]

for radar in config.get("radars", []):
    role = radar.get("role")
    dev_id = radar.get("id", "").strip()
    if not role or not dev_id:
        continue
    
    symlink_prefix = f"ti_{role.lower()}"
    is_path = "-" in dev_id or "." in dev_id
    
    if is_path:
        rules.append(f"# Role: {role} mapped to USB Physical Path: {dev_id}")
        # CLI Interface (Interface 00 / Subdirectory :1.0)
        rules.append(f"SUBSYSTEM==\"tty\", KERNELS==\"{dev_id}:1.0\", SYMLINK+=\"{symlink_prefix}_cli\", MODE=\"0666\", GROUP=\"dialout\"")
        # Data Interface (Interface 01 for CP2105, Interface 03 for XDS110 / Subdirectory :1.1 or :1.3)
        rules.append(f"SUBSYSTEM==\"tty\", KERNELS==\"{dev_id}:1.1\", SYMLINK+=\"{symlink_prefix}_data\", MODE=\"0666\", GROUP=\"dialout\"")
        rules.append(f"SUBSYSTEM==\"tty\", KERNELS==\"{dev_id}:1.3\", SYMLINK+=\"{symlink_prefix}_data\", MODE=\"0666\", GROUP=\"dialout\"")
    else:
        rules.append(f"# Role: {role} mapped to USB Serial: {dev_id}")
        # XDS110 Bridge (0451:bef3)
        rules.append(f"SUBSYSTEM==\"tty\", ATTRS{{serial}}==\"{dev_id}\", ATTRS{{idVendor}}==\"0451\", ATTRS{{idProduct}}==\"bef3\", ENV{{ID_USB_INTERFACE_NUM}}==\"00\", SYMLINK+=\"{symlink_prefix}_cli\", MODE=\"0666\", GROUP=\"dialout\"")
        rules.append(f"SUBSYSTEM==\"tty\", ATTRS{{serial}}==\"{dev_id}\", ATTRS{{idVendor}}==\"0451\", ATTRS{{idProduct}}==\"bef3\", ENV{{ID_USB_INTERFACE_NUM}}==\"03\", SYMLINK+=\"{symlink_prefix}_data\", MODE=\"0666\", GROUP=\"dialout\"")
        # CP2105 Bridge (10c4:ea70)
        rules.append(f"SUBSYSTEM==\"tty\", ATTRS{{serial}}==\"{dev_id}\", ATTRS{{idVendor}}==\"10c4\", ATTRS{{idProduct}}==\"ea70\", ENV{{ID_USB_INTERFACE_NUM}}==\"00\", SYMLINK+=\"{symlink_prefix}_cli\", MODE=\"0666\", GROUP=\"dialout\"")
        rules.append(f"SUBSYSTEM==\"tty\", ATTRS{{serial}}==\"{dev_id}\", ATTRS{{idVendor}}==\"10c4\", ATTRS{{idProduct}}==\"ea70\", ENV{{ID_USB_INTERFACE_NUM}}==\"01\", SYMLINK+=\"{symlink_prefix}_data\", MODE=\"0666\", GROUP=\"dialout\"")
    rules.append("")

with open(rules_path, "w") as f:
    f.write("\n".join(rules) + "\n")

print(f"TI Radar udev rules written to {rules_path}")
' "$CONFIG_FILE" "$RULES_FILE"

echo "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "TI Radar udev rules setup completed successfully."

