general feedback from testing:

1. on this machine the code works fine, but on another test machine, for the intel realsense, I was experiencing some odd issues where only the /dev/video ports were being detected, not the hid imu ports on the camera (even outside of the docker container). Also, when running the detect devices script, no serial number was detected, but the port was. Finally, the realsense ros2 driver was claiming that the realsense was running over usb2.0 when it was plugged into a usb3 interface. This machine did have usbguard on it so see if that might have an effect on it maybe? Look into resources that might help explain this issue.

2. When running the ros2 nodes on with the realsense (in the docker container), it seemed that the ros2 launch file wasn't actually detecting the parameters and was only outputting a depth map data stream instead of the full rgb, rgb + d, and depth topics. Test out the current docker implementation and confirm that these are actually streaming through and that they are working correctly. 

3. If the detect_devices.sh script is run before the docker image is created, it appeared that the .env with the interface documentation was being overidden/cleared (thus no interfaces were populated until the detect_devices script was run again). Check this and correct the behavior as needed. 

4. create special doccker .json files for the ti radars, ouster lidars, and livox lidars with the ports setup such that they use the /dev/ti ... nomenclature as well as that the ethernet ports for the dca1000 use the 0.0.0.0 ip address for the machine's static ip addresss (the fpga's is still the normla one, but the host ip address if needed is set to 0.0.0.0 so that it works with the docker containers)

5. For the docker compose dev variants, please still keep the full ports in the devices( i.e. devices:
      # TI Radars Serial Interfaces (Role Mapped)
      - "${FRONT_RADAR_CLI:-/dev/null}:${FRONT_RADAR_CLI:-/dev/null}"
      - "${FRONT_RADAR_DATA:-/dev/null}:${FRONT_RADAR_DATA:-/dev/null}"
      - "${BACK_RADAR_CLI:-/dev/null}:${BACK_RADAR_CLI:-/dev/null}"
      - "${BACK_RADAR_DATA:-/dev/null}:${BACK_RADAR_DATA:-/dev/null}"
      - "${DOWN_RADAR_CLI:-/dev/null}:${DOWN_RADAR_CLI:-/dev/null}"
      - "${DOWN_RADAR_DATA:-/dev/null}:${DOWN_RADAR_DATA:-/dev/null}"
      # Intel RealSense Camera Video Devices
      - "${REALSENSE_DEV_0:-/dev/null}:${REALSENSE_DEV_0:-/dev/null}"
      - "${REALSENSE_DEV_1:-/dev/null}:${REALSENSE_DEV_1:-/dev/null}"
      - "${REALSENSE_DEV_2:-/dev/null}:${REALSENSE_DEV_2:-/dev/null}"
      - "${REALSENSE_DEV_3:-/dev/null}:${REALSENSE_DEV_3:-/dev/null}"
      - "${REALSENSE_DEV_4:-/dev/null}:${REALSENSE_DEV_4:-/dev/null}"
      - "${REALSENSE_DEV_5:-/dev/null}:${REALSENSE_DEV_5:-/dev/null}"
      # Intel RealSense Camera IMU/HID Devices
      - "${REALSENSE_HID_0:-/dev/null}:${REALSENSE_HID_0:-/dev/null}"
)) as done for the regular. Only the networking setup should be different in this case. 

6. expand the detect devices to include all possible sensors (e.g. lidars, dca1000's if possible) [DON"T ACT ON THIS ONE YET]