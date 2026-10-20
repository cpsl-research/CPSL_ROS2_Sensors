# Tutorial 02: RealSense setup and debugging

Goal: make an Intel RealSense camera (and its IMU) usable from ROS on a plain Ubuntu host, and know how to check it when frames are slow or missing. Prerequisite: `bash scripts/install.sh --sensors realsense` ([Tutorial 00](00_ubuntu_to_first_topic.md)). Using the Docker image instead: the same host steps apply, then see [Tutorial 07](07_docker.md).

---

## 1. Check the USB connection

RealSense color plus depth needs USB 3 (SuperSpeed). Plug the camera in and run:

```bash
lsusb -t
```

Find the `Intel(R) RealSense(TM)` entry. Its port should show `5000M` or `10000M`; `480M` means USB 2, which streams too little data. If you see `480M`, flip the USB-C connector, use the cable that came with the camera, and plug straight into the machine instead of an unpowered hub.

## 2. Install the udev rules and enable the IMU

The librealsense built by `scripts/deps/realsense.sh` talks to the camera from user space, so your user needs permission to open it, and to enable the IMU channels in `/sys`. One helper checks and fixes both. Check first (read-only):

```bash
python3 tools/setup/realsense_host_setup.py
```

You should see three lines, `rules`, `camera` and `iio`, each `OK`, `MISSING`, `WARN` or `N-A`. A fresh host shows `rules` as `MISSING`, with the commands that would fix it. Preview what `--apply` runs, then run it (it asks once, then uses `sudo` per command, so run it as yourself):

```bash
python3 tools/setup/realsense_host_setup.py --apply --dry-run
python3 tools/setup/realsense_host_setup.py --apply
```

`--apply` copies the rules file shipped in `tools/setup/rules/` to `/etc/udev/rules.d/99-realsense-libusb.rules` (an existing different file is backed up first), reloads udev, re-triggers the camera and re-checks. It deletes nothing. `bash scripts/setup_realsense_udev.sh` is a shortcut for `--apply`. Unplug and replug the camera, then run the check again: all three should be `OK`. `iio` is the IMU: it passes when a non-root user can write the IMU's `scan_elements/*_en` files.

## 3. Look at the camera

The RealSense tools are installed with the SDK. List the device, then open the viewer:

```bash
rs-enumerate-devices
realsense-viewer
```

`rs-enumerate-devices` prints the camera name, firmware version, streams and the USB connection type. In the viewer, switch the Stereo Module, RGB Camera and Motion Module on in the side panel to see depth, color and IMU graphs.

## 4. Start it from ROS

[Tutorial 00](00_ubuntu_to_first_topic.md) section 6 starts the camera with `realsense_enable:=true`. That launch enables color, depth, aligned depth and a point cloud; it does not pass IMU options. To stream the IMU, start the upstream driver directly. Its argument names are defined by the `realsense2_camera` package, so list them first:

```bash
ros2 launch realsense2_camera rs_launch.py --show-args
```

The IMU arguments are `enable_gyro` and `enable_accel`; set both to `true` (and `unite_imu_method:=2` for one combined `imu` topic). If `iio` was not `OK` in step 2, no IMU stream appears at all.

## 5. Troubleshooting

**Frames at 1-5 Hz, or "Frames didn't arrive" warnings.** Linux gives USB devices a small 16 MB buffer by default; color plus depth overflows it. Raise it until reboot:

```bash
sudo sh -c 'echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

To keep it across reboots, add the option to a modprobe file and reboot:

```bash
echo "options usbcore usbfs_memory_mb=1024" | sudo tee /etc/modprobe.d/usbcore.conf
```

**Camera listed but no IMU.** Re-run step 2; `iio` must be `OK`. On a machine with USBGuard, the camera's HID interface may be blocked: `sudo usbguard list-devices` shows its state (vendor `8086`). Test with `sudo usbguard allow-device <id> --temp`; if that fixes it, make it permanent with `sudo usbguard allow-device <id>`.

**USB errors.** Watch the kernel log while replugging: `sudo dmesg -w`. Look for disconnects, enumeration failures and timeouts.

Inside Docker: see [Tutorial 07](07_docker.md) section 7.
