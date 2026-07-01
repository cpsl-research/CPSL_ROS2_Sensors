# Tutorial 07: RealSense Viewer Debugging in Docker

This tutorial covers how to verify RealSense camera connections, check USB speeds, and launch the graphical `realsense-viewer` utility from within the isolated Docker container.

---

## 1. Verifying Physical Connection on Host First

Before attempting to debug inside Docker, verify that the host machine recognizes the camera:

### A. Check USB Device Speed
RealSense cameras require USB 3.0 (SuperSpeed) bandwidth for color + depth alignment and streaming. Run the following command on the host:
```bash
lsusb -t
```
Look for the `Intel(R) RealSense(TM)` device in the tree. Ensure it is connected to a parent port showing `5000M` (5 Gbps / USB 3.0) or `10000M` (10 Gbps / USB 3.1), rather than `480M` (480 Mbps / USB 2.0).

*If it is running at USB 2.0 speed:*
- Unplug the cable, flip the USB-C connector orientation, and plug it back in.
- Ensure you are using the high-quality USB 3.0 cable provided with the camera.
- Connect directly to the motherboard rather than via an unpowered USB hub.
- Check if USBGuard is restricting the interface speed (see [Tutorial 06](file:///home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/tutorials/06_udev_setup.md)).

---

## 2. Launching realsense-viewer from the Docker Container

The Docker image is built with the full graphical Intel RealSense SDK and tools compiled from source via the RSUSB (libusb) user-space driver backend.

> [!IMPORTANT]
> **Use the Kernel-Backend Diagnostics (V4L2):**
> Because the source-compiled SDK (in `/usr/local/bin`) uses the `RSUSB` backend, it requires direct, raw access to the host's `/dev/bus/usb` which is blocked in non-privileged Docker sandboxes.
> To run the diagnostic tools securely without privileged mode, you **must use the APT-installed versions** located under `/opt/ros/jazzy/bin/` (e.g., `/opt/ros/jazzy/bin/realsense-viewer` and `/opt/ros/jazzy/bin/rs-enumerate-devices`). These utilize the standard Linux kernel V4L2 backend and pair successfully using the character device nodes passed into the container.

### Step 1: Configure X11 Forwarding Permissions on Host
To allow GUI applications from the container to display on your monitor, authorize X11 access on the host:
```bash
xhost +local:root
```

### Step 2: Launch the Viewer
Run the viewer container using your preferred docker-compose target.

For CPU configurations:
```bash
docker compose -f docker/docker-compose.cpu.yaml run --rm cpsl_sensors /opt/ros/jazzy/bin/realsense-viewer
```

For GPU configurations:
```bash
docker compose -f docker/docker-compose.gpu.yaml run --rm cpsl_sensors /opt/ros/jazzy/bin/realsense-viewer
```

This will launch the graphical realsense-viewer window. You can toggle the stereo depth module, RGB module, and Motion (IMU) module from the side panel to confirm they are capturing frames.

---

## 3. Command Line Diagnostic Utilities

If you do not have an active X11 display server or want a text-based status report:

### A. List connected devices & profiles
Run `/opt/ros/jazzy/bin/rs-enumerate-devices` inside the container:
```bash
docker compose -f docker/docker-compose.cpu.yaml run --rm cpsl_sensors /opt/ros/jazzy/bin/rs-enumerate-devices
```
This utility will output:
- Detected device names and firmware versions.
- Supported camera streams and frame rates.
- The connection protocol speed (USB 3.2 vs USB 2.1).

### B. Print hardware error logs
To check for hardware-level handshake issues, check the kernel messages inside the container or host:
```bash
dmesg -w
```
Look for USB disconnects, enumeration failures, or UVC timeouts.

---

## 4. Troubleshooting Low Framerates & Startup Crashes inside Docker

When streaming the camera inside isolated Docker containers, you may encounter the following common issues:

### A. Low Framerate (1-5 Hz) or "Frames didn't arrive" Warnings
The Linux kernel defaults to a tiny **16MB** USB FS memory buffer (`usbfs_memory_mb`). High-bandwidth USB 3.0 streaming (simultaneous depth + color) inside a container will quickly overflow this buffer, causing packet drops and frame timeouts.

**The Solution:**
Temporarily increase the USB FS memory buffer to **1024MB** on the host machine:
```bash
sudo sh -c 'echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```
To make this change permanent across reboots, add the parameter to your host's GRUB configuration or `/etc/modprobe.d/`:
```bash
echo "options usbcore usbfs_memory_mb=1024" | sudo tee /etc/modprobe.d/usbcore.conf
```

### B. Startup Crash with "Read-only file system" Error
If the IMU (gyroscope and accelerometer) streams are enabled, `librealsense` attempts to write configuration values (`1` to enable) directly to the host's sysfs IIO directories (e.g. `/sys/devices/.../iio:device1/scan_elements/in_anglvel_x_en`). Because `/sys` is mounted read-only inside Docker containers for security, the driver fails with `Read-only file system` and crashes.

**The Workaround:**
Explicitly disable the IMU streams using the launch parameters, which tells the driver to completely skip the sysfs configuration:
```bash
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py \
  realsense_enable:=true \
  enable_gyro:=false \
  enable_accel:=false
```
*(If IMU data is required, you must mount `/sys/devices` as read-write (`rw`) AND configure `security_opt: ["apparmor=unconfined"]` inside the docker-compose configuration files, which is done by default. Attempting to mount only specific subdirectories for the RealSense device (e.g. `/sys/devices/pci0000:00/...`) or omitting the AppArmor security option will fail due to:
1. **Docker Delimiter Parsing Conflict:** Docker interprets any colon (`:`) as a field separator in volume mounts (even in long syntax binds), causing daemon-level parsing failure on PCI paths (like `pci0000:00`).
2. **SDK Association Constraints:** The RealSense SDK matches cameras and IMU devices by walking up the host `/sys/devices` path structure to check parent descriptors. Mapping target paths to customized locations without colons breaks this pairing logic, disabling the IMU stream.
3. **AppArmor Write Restrictions:** By default, Docker's default AppArmor profile blocks all write operations under `/sys/**` inside containers. Setting `security_opt: ["apparmor=unconfined"]` unconfines AppArmor for the container. Since other directories in `/sys` are still mounted read-only by Docker's default layout, the write capability remains securely limited only to our read-write `/sys/devices` volume mount.
Hence, mounting the entire `/sys/devices` folder as `rw` with unconfined AppArmor is structurally and operationally necessary for IMU support).*

