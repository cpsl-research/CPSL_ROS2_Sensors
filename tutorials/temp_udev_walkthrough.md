# Temporary Interactive udev Rules Walkthrough Sheet (Option B Mappings)

This temporary worksheet is designed to guide you through setting up and verifying the **Option B (host-level symlinks)** udev rules step-by-step.

Run each command in your host terminal, paste the corresponding output in the marked code blocks below, and save the file. Antigravity will examine the results to verify that the host symlinks are resolved and bound correctly.

---

## Step 0: Find Connected Device IDs and Edit Config

Before generating udev rules, you need to find the unique identifier (Serial Number or physical USB path) of your connected devices and write them to `docker/device_config.json`.

1. Run the raw device discovery script to print currently connected device info:
   ```bash
   ./scripts/detect_devices.sh
   ```

### 📋 Paste Output of Step 0 Here:
```text
./scripts/detect_devices.sh                                             david@thinkpadP15v3
=== Running Host Device Discovery ===
Loading device configuration from: /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker/device_config.json
Scanning host for connected TI Radars...
Found 1 connected TI Radar device(s).
  - Serial: 019C697F, Path: 3-7, CLI: /dev/ttyUSB0, Data: /dev/ttyUSB1
Scanning host for connected Intel RealSense cameras...
Found 1 connected RealSense camera(s).
  - Serial: 251343061404, Path: 4-1
    Video Ports: /dev/video4, /dev/video5, /dev/video6, /dev/video7, /dev/video8, /dev/video9
    HID Ports:   /dev/hidraw0, /dev/iio:device0, /dev/iio:device1
Auto-mapped connected Radar (Serial: 019C697F, Path: 3-7) to role FRONT_RADAR (raw ports: /dev/ttyUSB0)
Auto-mapped connected RealSense (Serial: 251343061404, Path: 4-1) to REALSENSE role
Successfully updated env file: /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker/.env
Successfully updated env file: /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/.env
=== Host Device Discovery Completed ===

```

2. **Action**: Open `docker/device_config.json` and edit it to map your devices:
   - Under `radars`, set the `id` field of the role (e.g. `"role": "FRONT_RADAR"`) to either the **Serial** (e.g., `"id": "019C697F"`) or the **USB Path** (e.g., `"id": "3-1"`) found in the step 0 output.
   - Under `realsense`, set the `id` field to the **Serial** (e.g., `"id": "251343061404"`) or **USB Path** (e.g., `"id": "4-2"`).

Save the modified `device_config.json` file before moving to Step 1.

---

## Step 1: Generate & Install TI Radar udev Rules

Run the updated setup script. This script reads your `docker/device_config.json` configuration and writes serial/port path specific rules to generate custom symlinks (e.g. `/dev/ti_front_radar_cli`):
```bash
sudo ./scripts/setup_radar_udev.sh
```

### 📋 Paste Output of Step 1 Here:
```text
sudo ./scripts/setup_radar_udev.sh                                      david@thinkpadP15v3
Generating TI Radar udev rules from /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/scripts/../docker/device_config.json...
TI Radar udev rules written to /etc/udev/rules.d/98-ti-radar.rules
Reloading udev rules...
TI Radar udev rules setup completed successfully.

```

---

## Step 2: Install Intel RealSense udev Rules

Run the RealSense udev configuration script to configure permissions for depth and IMU devices:
```bash
sudo ./scripts/setup_realsense_udev.sh
```

### 📋 Paste Output of Step 2 Here:
```text
sudo ./scripts/setup_realsense_udev.sh                              1 ↵ david@thinkpadP15v3
Setting up Intel RealSense udev rules...
Attempting to download rules via curl...
Successfully downloaded official Intel RealSense rules.
Reloading udev rules...
Intel RealSense udev rules setup completed successfully.

```

---

## Step 3: Replug Devices and Check Symlinks / Node Permissions

Unplug both your TI Radar and Intel RealSense USB cables from the host machine, wait 2 seconds, and plug them back in. Then run the following commands to verify:
1. The custom `/dev/ti_*` symlinks have been successfully created.
2. The nodes have open read/write (`crw-rw-rw-`) permissions.

```bash
# Check TI Radar Symlinks
ls -l /dev/ti_* 2>/dev/null

# Check TI Radar raw port permissions
ls -l /dev/ttyUSB*  2>/dev/null

ls -l /dev/ttyACM*  2>/dev/null

# Check RealSense video, HID, and IMU permissions
ls -l /dev/video* /dev/hidraw* /dev/iio:device* 2>/dev/null
```

### 📋 Paste Output of Step 3 Here:
```text
# Check TI Radar Symlinks                                               david@thinkpadP15v3
ls -l /dev/ti_* 2>/dev/null

# Check TI Radar raw port permissions
ls -l /dev/ttyUSB*  2>/dev/null

ls -l /dev/ttyACM*  2>/dev/null

# Check RealSense video, HID, and IMU permissions
ls -l /dev/video* /dev/hidraw* /dev/iio:device* 2>/dev/null
lrwxrwxrwx 1 root root 7 Jun 28 21:30 /dev/ti_front_radar_cli -> ttyUSB0
lrwxrwxrwx 1 root root 7 Jun 28 21:30 /dev/ti_front_radar_data -> ttyUSB1
crw-rw-rw- 1 root plugdev 188, 0 Jun 28 21:30 /dev/ttyUSB0
crw-rw-rw- 1 root plugdev 188, 1 Jun 28 21:30 /dev/ttyUSB1
zsh: no matches found: /dev/ttyACM*
crw-rw-rw-  1 root plugdev 508, 0 Jun 28 21:29 /dev/hidraw0
crw-rw-rw-  1 root plugdev 507, 0 Jun 28 21:29 /dev/iio:device0
crw-rw-rw-  1 root plugdev 507, 1 Jun 28 21:29 /dev/iio:device1
crw-rw----+ 1 root plugdev  81, 0 Jun 28 21:29 /dev/video0
crw-rw----+ 1 root plugdev  81, 1 Jun 28 21:29 /dev/video1
crw-rw----+ 1 root plugdev  81, 2 Jun 28 21:29 /dev/video2
crw-rw----+ 1 root plugdev  81, 3 Jun 28 21:29 /dev/video3
crw-rw-rw-+ 1 root plugdev  81, 4 Jun 28 21:29 /dev/video4
crw-rw-rw-+ 1 root plugdev  81, 5 Jun 28 21:29 /dev/video5
crw-rw-rw-+ 1 root plugdev  81, 6 Jun 28 21:29 /dev/video6
crw-rw-rw-+ 1 root plugdev  81, 7 Jun 28 21:29 /dev/video7
crw-rw-rw-+ 1 root plugdev  81, 8 Jun 28 21:29 /dev/video8
crw-rw-rw-+ 1 root plugdev  81, 9 Jun 28 21:29 /dev/video9
```

---

## Step 4: Run Standalone Device Discovery

Run the discovery script. Under Option B, this script will detect that the udev symlinks point to the connected radar and write the symlink paths directly into your `.env` configuration file:
```bash
./scripts/detect_devices.sh
```

### 📋 Paste Output of Step 4 Here:
```text
sudo ./scripts/detect_devices.sh                                        david@thinkpadP15v3
=== Running Host Device Discovery ===
Loading device configuration from: /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker/device_config.json
Scanning host for connected TI Radars...
Found 1 connected TI Radar device(s).
  - Serial: 019C697F, Path: 3-7, CLI: /dev/ttyUSB0, Data: /dev/ttyUSB1
Scanning host for connected Intel RealSense cameras...
Found 1 connected RealSense camera(s).
  - Serial: 251343061404, Path: 4-1
    Video Ports: /dev/video4, /dev/video5, /dev/video6, /dev/video7, /dev/video8, /dev/video9
    HID Ports:   /dev/hidraw0, /dev/iio:device0, /dev/iio:device1
Mapped Radar 3-7 to role FRONT_RADAR using symlinks /dev/ti_front_radar_cli -> /dev/ttyUSB0
Mapped RealSense 4-1 to REALSENSE role
Successfully updated env file: /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker/.env
Successfully updated env file: /home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/.env
=== Host Device Discovery Completed ===
```
