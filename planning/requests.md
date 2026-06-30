general feedback from testing:

1. When running the realsense-viewer: I'm getting the following error:
Backend in rs2_open_multiple(sensor:0x764670020c40,
profiles:0x7646705572b8, count:2):
Failed to open scan_element
/sys/devices/pci0000:00/0000:00:14.0/usb4/4-1/4-1:1.5/0003:8086:0B3A.000D/HID-SENSOR-200076.3.auto/iio:device1/scan_elements/in_anglvel_x_en
Last Error: Read-only file system

2. When attempting to launch rviz2 (or any other gui), I get this error: rviz2
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
MESA: error: Failed to query drm device.
glx: failed to create dri3 screen
failed to load driver: iris


6. expand the detect devices to include all possible sensors (e.g. lidars, dca1000's if possible) [DON"T ACT ON THIS ONE YET]