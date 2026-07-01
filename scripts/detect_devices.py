#!/usr/bin/env python3
import os
import json
import re
import sys

def get_parent_usb_device(dev_path):
    """Walk up the sysfs device path to find parent USB device folder with idVendor and idProduct."""
    real_path = os.path.realpath(dev_path)
    curr = real_path
    
    # Try to find bInterfaceNumber along the way
    bInterfaceNumber = None
    if "device" in os.listdir(tty_dir_for(real_path)):
        ifos_dir = os.path.join(tty_dir_for(real_path), "device")
        bif_path = os.path.join(ifos_dir, "bInterfaceNumber")
        if os.path.exists(bif_path):
            try:
                with open(bif_path, "r") as f:
                    bInterfaceNumber = f.read().strip()
            except Exception:
                pass

    # Now find USB parent device root (contains idVendor and idProduct)
    while curr and curr != "/":
        if os.path.exists(os.path.join(curr, "idVendor")) and os.path.exists(os.path.join(curr, "idProduct")):
            try:
                with open(os.path.join(curr, "idVendor"), "r") as f:
                    vid = f.read().strip().lower()
                with open(os.path.join(curr, "idProduct"), "r") as f:
                    pid = f.read().strip().lower()
                serial = ""
                if os.path.exists(os.path.join(curr, "serial")):
                    with open(os.path.join(curr, "serial"), "r") as f:
                        serial = f.read().strip()
                usb_path = os.path.basename(curr)
                return {
                    "vid": vid,
                    "pid": pid,
                    "serial": serial,
                    "usb_path": usb_path,
                    "bInterfaceNumber": bInterfaceNumber
                }
            except Exception:
                pass
        curr = os.path.dirname(curr)
    return None

def tty_dir_for(real_path):
    # If the real path is a class device, it has no nested class dir.
    return real_path

def find_radar_devices():
    """Find all connected TI Radar ports (XDS110 and CP2105)."""
    radars = {}
    tty_base = "/sys/class/tty"
    if not os.path.exists(tty_base):
        return radars

    for dev in os.listdir(tty_base):
        if not (dev.startswith("ttyACM") or dev.startswith("ttyUSB")):
            continue
        dev_path = os.path.join(tty_base, dev)
        info = get_parent_usb_device(dev_path)
        if not info:
            continue
        
        # Check for XDS110 (0451:bef3) or CP2105 (10c4:ea70)
        is_xds110 = (info["vid"] == "0451" and info["pid"] == "bef3")
        is_cp2105 = (info["vid"] == "10c4" and info["pid"] == "ea70")
        
        if is_xds110 or is_cp2105:
            device_key = info["serial"] if info["serial"] else info["usb_path"]
            if device_key not in radars:
                radars[device_key] = {
                    "serial": info["serial"],
                    "usb_path": info["usb_path"],
                    "vid": info["vid"],
                    "pid": info["pid"],
                    "ports": []
                }
            radars[device_key]["ports"].append({
                "dev_node": f"/dev/{dev}",
                "interface": info["bInterfaceNumber"]
            })
            
    # Resolve CLI and Data ports for each radar device
    resolved_radars = []
    for key, info in radars.items():
        cli_port = None
        data_port = None
        
        # Identify by interface number
        for port in info["ports"]:
            if port["interface"] == "00":
                cli_port = port["dev_node"]
            elif info["vid"] == "0451" and port["interface"] == "03":
                data_port = port["dev_node"]
            elif info["vid"] == "10c4" and port["interface"] == "01":
                data_port = port["dev_node"]
                
        # Fallback to sorting ports if interface numbers didn't match expected values
        if not cli_port or not data_port:
            sorted_ports = sorted([p["dev_node"] for p in info["ports"]])
            if len(sorted_ports) >= 2:
                cli_port = sorted_ports[0]
                data_port = sorted_ports[1]
            elif len(sorted_ports) == 1:
                cli_port = sorted_ports[0]
                
        resolved_radars.append({
            "serial": info["serial"],
            "usb_path": info["usb_path"],
            "cli_port": cli_port,
            "data_port": data_port
        })
    return resolved_radars

def find_realsense_devices():
    """Find all connected Intel RealSense video and IMU/HID ports."""
    realsenses = {}
    
    # 1. Search for video devices
    v4l_base = "/sys/class/video4linux"
    if os.path.exists(v4l_base):
        for dev in os.listdir(v4l_base):
            dev_path = os.path.join(v4l_base, dev)
            info = get_parent_usb_device(dev_path)
            if not info:
                continue
            
            # Intel VID is 8086 or 086a
            if info["vid"] in ["8086", "086a"]:
                device_key = info["serial"] if info["serial"] else info["usb_path"]
                if device_key not in realsenses:
                    realsenses[device_key] = {
                        "serial": info["serial"],
                        "usb_path": info["usb_path"],
                        "video_ports": [],
                        "hid_ports": []
                    }
                realsenses[device_key]["video_ports"].append(f"/dev/{dev}")

    # 2. Search for HID raw devices (for RealSense IMUs)
    hid_base = "/sys/class/hidraw"
    if os.path.exists(hid_base):
        for dev in os.listdir(hid_base):
            dev_path = os.path.join(hid_base, dev)
            info = get_parent_usb_device(dev_path)
            if not info:
                continue
            if info["vid"] in ["8086", "086a"]:
                device_key = info["serial"] if info["serial"] else info["usb_path"]
                if device_key in realsenses:
                    realsenses[device_key]["hid_ports"].append(f"/dev/{dev}")
                else:
                    # In case we only found hidraw but no video, initialize it
                    realsenses[device_key] = {
                        "serial": info["serial"],
                        "usb_path": info["usb_path"],
                        "video_ports": [],
                        "hid_ports": [f"/dev/{dev}"]
                    }
                    
    # 3. Search for IIO devices (alternative IMU interface)
    iio_base = "/sys/bus/iio/devices"
    if os.path.exists(iio_base):
        for dev in os.listdir(iio_base):
            if not dev.startswith("iio:device"):
                continue
            dev_path = os.path.join(iio_base, dev)
            info = get_parent_usb_device(dev_path)
            if not info:
                continue
            if info["vid"] in ["8086", "086a"]:
                device_key = info["serial"] if info["serial"] else info["usb_path"]
                if device_key in realsenses:
                    realsenses[device_key]["hid_ports"].append(f"/dev/{dev}")
                else:
                    realsenses[device_key] = {
                        "serial": info["serial"],
                        "usb_path": info["usb_path"],
                        "video_ports": [],
                        "hid_ports": [f"/dev/{dev}"]
                    }
                    
    resolved_realsenses = []
    for key, info in realsenses.items():
        # Sort video and hid ports numerically
        def sort_key(path):
            digits = re.findall(r"\d+", path)
            return int(digits[0]) if digits else 0
            
        resolved_realsenses.append({
            "serial": info["serial"],
            "usb_path": info["usb_path"],
            "video_ports": sorted(info["video_ports"], key=sort_key),
            "hid_ports": sorted(info["hid_ports"], key=sort_key)
        })
    return resolved_realsenses

def load_device_config(config_path):
    """Load configuration mappings from device_config.json."""
    if not os.path.exists(config_path):
        # Create default config if missing
        default_config = {
            "radars": [
                {"role": "FRONT_RADAR", "id": ""},
                {"role": "BACK_RADAR", "id": ""},
                {"role": "DOWN_RADAR", "id": ""}
            ],
            "realsense": {
                "role": "REALSENSE",
                "id": ""
            }
        }
        try:
            with open(config_path, "w") as f:
                json.dump(default_config, f, indent=2)
        except Exception as e:
            print(f"Error creating default config: {e}")
        return default_config
        
    try:
        with open(config_path, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"Error reading device config: {e}")
        return None

def resolve_device_roles(connected_radars, connected_realsenses, config):
    """Resolve roles (FRONT_RADAR, etc.) based on serial, USB path, or default fallback mapping."""
    mappings = {
        "FRONT_RADAR_CLI": "/dev/null",
        "FRONT_RADAR_DATA": "/dev/null",
        "BACK_RADAR_CLI": "/dev/null",
        "BACK_RADAR_DATA": "/dev/null",
        "DOWN_RADAR_CLI": "/dev/null",
        "DOWN_RADAR_DATA": "/dev/null",
        "HOST_DRI_PATH": "/dev/null",
    }
    for i in range(6):
        mappings[f"REALSENSE_DEV_{i}"] = "/dev/null"
    for i in range(3):
        mappings[f"REALSENSE_HID_{i}"] = "/dev/null"

    if os.path.exists("/dev/dri"):
        mappings["HOST_DRI_PATH"] = "/dev/dri"

    # 1. Map Radars
    radar_config = config.get("radars", [])
    used_radars = set()
    
    # First pass: match explicit IDs (serial number or USB path)
    for rc in radar_config:
        role = rc.get("role")
        target_id = rc.get("id", "").strip()
        if not target_id:
            continue
            
        for radar in connected_radars:
            radar_ref = f"{radar['cli_port']}:{radar['data_port']}"
            if radar_ref in used_radars:
                continue
            if target_id == radar["serial"] or target_id == radar["usb_path"]:
                sym_cli = f"/dev/ti_{role.lower()}_cli"
                sym_data = f"/dev/ti_{role.lower()}_data"
                if os.path.exists(sym_cli) and os.path.realpath(sym_cli) == os.path.realpath(radar["cli_port"]):
                    mappings[f"{role}_CLI"] = sym_cli
                    mappings[f"{role}_DATA"] = sym_data
                    print(f"Mapped Radar {target_id} to role {role} using symlinks {sym_cli} -> {radar['cli_port']}")
                else:
                    mappings[f"{role}_CLI"] = radar["cli_port"]
                    mappings[f"{role}_DATA"] = radar["data_port"]
                    print(f"Mapped Radar {target_id} to role {role} (raw ports: {radar['cli_port']})")
                used_radars.add(radar_ref)
                break

    # Second pass: map unassigned connected radars to unassigned roles sequentially
    for rc in radar_config:
        role = rc.get("role")
        cli_key = f"{role}_CLI"
        if mappings.get(cli_key) != "/dev/null":
            continue # already mapped
            
        # Find first unassigned connected radar
        for radar in connected_radars:
            radar_ref = f"{radar['cli_port']}:{radar['data_port']}"
            if radar_ref not in used_radars:
                sym_cli = f"/dev/ti_{role.lower()}_cli"
                sym_data = f"/dev/ti_{role.lower()}_data"
                if os.path.exists(sym_cli) and os.path.realpath(sym_cli) == os.path.realpath(radar["cli_port"]):
                    mappings[cli_key] = sym_cli
                    mappings[f"{role}_DATA"] = sym_data
                    print(f"Auto-mapped connected Radar (Serial: {radar['serial']}, Path: {radar['usb_path']}) to role {role} using symlinks {sym_cli} -> {radar['cli_port']}")
                else:
                    mappings[cli_key] = radar["cli_port"]
                    mappings[f"{role}_DATA"] = radar["data_port"]
                    print(f"Auto-mapped connected Radar (Serial: {radar['serial']}, Path: {radar['usb_path']}) to role {role} (raw ports: {radar['cli_port']})")
                used_radars.add(radar_ref)
                break

    # 2. Map RealSense
    realsense_config = config.get("realsense", {})
    target_cam_id = realsense_config.get("id", "").strip()
    
    mapped_realsense = None
    
    if target_cam_id:
        # Match explicit ID
        for rs in connected_realsenses:
            if target_cam_id == rs["serial"] or target_cam_id == rs["usb_path"]:
                mapped_realsense = rs
                print(f"Mapped RealSense {target_cam_id} to REALSENSE role")
                break
    elif connected_realsenses:
        # Auto-map first connected
        mapped_realsense = connected_realsenses[0]
        print(f"Auto-mapped connected RealSense (Serial: {mapped_realsense['serial']}, Path: {mapped_realsense['usb_path']}) to REALSENSE role")
        
    if mapped_realsense:
        # Map video nodes
        for idx, port in enumerate(mapped_realsense["video_ports"][:6]):
            mappings[f"REALSENSE_DEV_{idx}"] = port
        # Map HID nodes (IMU)
        for idx, port in enumerate(mapped_realsense["hid_ports"][:3]):
            mappings[f"REALSENSE_HID_{idx}"] = port

    return mappings

def update_env_file(env_path, device_mappings):
    """Read existing .env file, preserve user configs, update device mappings, and write back."""
    env_data = {}
    
    # Read existing
    if os.path.exists(env_path):
        try:
            with open(env_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    if "=" in line:
                        k, v = line.split("=", 1)
                        env_data[k.strip()] = v.strip()
        except Exception as e:
            print(f"Warning reading env file {env_path}: {e}")

    # Update with device mappings
    env_data.update(device_mappings)
    
    # Write back
    try:
        with open(env_path, "w") as f:
            f.write("# Auto-generated device mappings\n")
            # Write devices first
            for k in sorted(device_mappings.keys()):
                f.write(f"{k}={device_mappings[k]}\n")
            f.write("\n# System parameters\n")
            # Write system params
            for k in sorted(env_data.keys()):
                if k not in device_mappings:
                    f.write(f"{k}={env_data[k]}\n")
        print(f"Successfully updated env file: {env_path}")
    except Exception as e:
        print(f"Error writing env file {env_path}: {e}")

def main():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    workspace_root = os.path.dirname(script_dir)
    
    config_path = os.path.join(workspace_root, "docker", "device_config.json")
    print(f"Loading device configuration from: {config_path}")
    config = load_device_config(config_path)
    
    if not config:
        print("Failed to load device config. Exiting.")
        sys.exit(1)
        
    print("Scanning host for connected TI Radars...")
    radars = find_radar_devices()
    print(f"Found {len(radars)} connected TI Radar device(s).")
    for r in radars:
        print(f"  - Serial: {r['serial']}, Path: {r['usb_path']}, CLI: {r['cli_port']}, Data: {r['data_port']}")
        
    print("Scanning host for connected Intel RealSense cameras...")
    realsenses = find_realsense_devices()
    print(f"Found {len(realsenses)} connected RealSense camera(s).")
    for rs in realsenses:
        print(f"  - Serial: {rs['serial']}, Path: {rs['usb_path']}")
        print(f"    Video Ports: {', '.join(rs['video_ports'])}")
        print(f"    HID Ports:   {', '.join(rs['hid_ports'])}")
        
    device_mappings = resolve_device_roles(radars, realsenses, config)
    
    # Update both env files
    env_paths = [
        os.path.join(workspace_root, "docker", ".env"),
        os.path.join(workspace_root, ".env")
    ]
    
    for path in env_paths:
        update_env_file(path, device_mappings)

if __name__ == "__main__":
    main()
