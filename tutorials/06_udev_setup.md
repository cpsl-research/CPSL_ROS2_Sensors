# Tutorial 06: Host udev Rules Setup and Device Mapping

This tutorial covers how to configure non-privileged device access on the host system, map physical USB sensors to specific roles (like `FRONT_RADAR` or `REALSENSE` camera) using Option B host-level symlinks, and bring up all platform sensors using the unified bringup templates.

---

## 1. Why Setup udev Rules?

By default, Linux limits read/write permissions for raw serial and USB interface nodes (like `/dev/ttyACM*`, `/dev/ttyUSB*`, and `/dev/bus/usb/*`) to the `root` user or members of the `dialout`/`plugdev` groups. 

To allow non-privileged docker containers to access connected TI Radars and RealSense cameras without running in **privileged mode** (which is a significant security risk and exposes the entire host `/dev` directory), we install custom **udev rules** on the host.

---

## 2. Pre-requisite: Identifying Connected Device IDs

Before configuring udev, you must find the unique identifier (Serial Number or physical USB hub port path) of your connected devices.

### Step A: Run Raw Discovery Scanner
Ensure your devices are plugged in, and execute the discovery script:
```bash
./scripts/detect_devices.sh
```

*Example Output:*
```text
Scanning host for connected TI Radars...
Found 1 connected TI Radar device(s).
  - Serial: 019C697F, Path: 3-7, CLI: /dev/ttyUSB0, Data: /dev/ttyUSB1
Scanning host for connected Intel RealSense cameras...
Found 1 connected RealSense camera(s).
  - Serial: 251343061404, Path: 4-1
```
This tells you:
- TI Radar Serial: `019C697F` (USB Path: `3-7`)
- RealSense Serial: `251343061404` (USB Path: `4-1`)

### Step B: Configure Roles in `device_config.json`
Open the configuration file at 📁 **`docker/device_config.json`** and map these IDs to roles:

*   **Mapping by Serial Number (Recommended for single-machine setups)**:
    ```json
    {
      "radars": [
        {
          "role": "FRONT_RADAR",
          "id": "019C697F"
        }
      ],
      "realsense": {
        "role": "REALSENSE",
        "id": "251343061404"
      }
    }
    ```
*   **Mapping by USB Physical Path (Recommended for multi-sensor or deployment platforms)**:
    If you want the front radar to always represent whatever is plugged into a specific USB port regardless of the board serial, map using the path:
    ```json
    {
      "radars": [
        {
          "role": "FRONT_RADAR",
          "id": "3-7"
        }
      ],
      "realsense": {
        "role": "REALSENSE",
        "id": "4-1"
      }
    }
    ```

---

## 3. Installing the Host udev Rules

Now that `device_config.json` is configured, run the setup scripts as `root` (using `sudo`).

### A. TI Radar Rules Setup
The setup script reads `device_config.json` dynamically and generates serial/port-specific rules. For mapped radars, it automatically creates descriptive host symlinks (e.g., `/dev/ti_front_radar_cli` and `/dev/ti_front_radar_data`) and unlocks `0666` read/write permissions.

```bash
sudo ./scripts/setup_radar_udev.sh
```

### B. Intel RealSense Rules Setup
The RealSense setup script inlines the official Intel rules to configure permissions for depth, RGB video, and internal HID/IIO (IMU) sensors:
```bash
sudo ./scripts/setup_realsense_udev.sh
```

---

## 4. Replugging and Verifying Mappings

For udev to apply the newly written rules, **unplug the TI Radar and Intel RealSense USB cables, wait 2 seconds, and plug them back in.**

### A. Verify Symlinks and Permissions
Check if the TI Radar symlinks were created and target the active tty device nodes:
```bash
ls -l /dev/ti_*
```
*Expected Output:*
```text
lrwxrwxrwx 1 root root 7 Jun 28 21:30 /dev/ti_front_radar_cli -> ttyUSB0
lrwxrwxrwx 1 root root 7 Jun 28 21:30 /dev/ti_front_radar_data -> ttyUSB1
```

Confirm raw permissions are unlocked (`crw-rw-rw-`) for all ports:
```bash
ls -l /dev/ttyUSB* /dev/video4 /dev/hidraw0 /dev/iio:device*
```

*(Note: RealSense cameras expose multiple nodes—video, hidraw, and iio—so udev only updates their permissions. Role mapping for RealSense is handled dynamically in user-space via the discovery script).*

---

## 5. Running the Device Discovery Script

With the udev symlinks created, run the discovery script again:
```bash
./scripts/detect_devices.sh
```

The script will detect that the `/dev/ti_*` symlinks target the active hardware and will write the symlinks directly to the environment file:
*   `FRONT_RADAR_CLI=/dev/ti_front_radar_cli`
*   `FRONT_RADAR_DATA=/dev/ti_front_radar_data`

This guarantees that the Docker containers receive stable symlink targets, and the Compose configuration remains completely identical and uniform across all developer machines.

---

## 6. Advanced Debugging & Profiling Custom Boards

If you plug in a custom TI Radar board or a new bridge card, you can inspect it manually:

### Find Vendor ID (VID) and Product ID (PID)
```bash
lsusb
```
*Example Output:*
```text
Bus 003 Device 004: ID 0451:bef3 Texas Instruments XDS110 Probe
```
Here, the VID is `0451` and the PID is `bef3`.

### Find Interface Numbers
TI radars stream on separate interfaces (typically Interface `00` for CLI and `01` or `03` for Data). To query the active port interface:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep bInterfaceNumber
```

### Find parent physical USB Port Path
To find which USB hub path is associated with a node:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep "looking at parent device"
```
Or check the kernel boot logs:
```bash
dmesg | grep tty
```
Look for lines like `usb 3-7: CP2105 converter now attached to ttyUSB0` where `3-7` is the physical hub port.

---

## 7. Secure Device Passthrough & Dynamic RealSense IMU Mapping (Strategy A)

To maintain host security, we **do not** bind-mount the host's `/dev` directory into the container. Instead, we use **Strategy A** (Dynamic Container-Side node creation via Sysfs).

### A. How it is Implemented
1. **Host Isolation**: The docker compose configuration files do not mount the host `/dev` directory. The container's `/dev` directory is a private sandbox managed by Docker.
2. **Whitelisting Permissions**: The compose files include a cgroup whitelist rule:
   ```yaml
   device_cgroup_rules:
     - 'c *:* rmw'
   ```
   This rule tells the Linux kernel to authorize character device access. It updates permission boundaries but does not create or expose any host files.
3. **Dynamic Node Creation (`mknod`)**: Inside [docker_entrypoint.sh](file:///home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker/docker_entrypoint.sh), when the container boots, it scans `/sys/bus/iio/devices` (which is mounted by default). For each device (e.g. `iio:device0`), the entrypoint:
   - Reads the dynamic major/minor number (e.g., `507:0`) from `/sys/bus/iio/devices/iio:deviceX/dev`.
   - Runs `mknod /dev/iio:deviceX c <major> <minor>` to create the corresponding character device node inside the container's private `/dev`.
   - This bypasses the OCI runtime/runc limitation where passing devices with colons (like `iio:device0`) inside the `devices` block causes Docker Compose to fail.

### B. Scaling to Multiple RealSense Cameras
If you connect multiple RealSense cameras, the dynamic architecture scales **automatically** without modifying the entrypoint script. Here is how you configure it:

1. **Host Scan & Config**: Run `./scripts/detect_devices.sh`. It will list all connected cameras and their serials. Add them under `docker/device_config.json`.
2. **Compose Variables**: The `detect_devices` script will bind the video and HID nodes to environment variables. For multiple cameras, you would add extra environment slots (e.g., `REALSENSE_DEV_6` to `REALSENSE_DEV_11` and `REALSENSE_HID_1` to `REALSENSE_HID_2`) in:
   - The `.env` file.
   - The `devices:` mapping section of the Compose files.
3. **Automatic IMU Mapping**: When the container boots, the entrypoint script loops through *every* device found in `/sys/bus/iio/devices/iio:device*`. If there are 2 cameras (exposing 4 IIO devices: `iio:device0` to `iio:device3`), it will automatically create all 4 character nodes inside the container. No manual modifications to the entrypoint are needed!

