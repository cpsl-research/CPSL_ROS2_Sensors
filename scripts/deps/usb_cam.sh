#!/usr/bin/env bash
# USB (UVC) cameras: the apt-packaged ROS driver, no submodule. Access to
# /dev/video* (the `video` group) is host user setup and lives in install.sh.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure ros-jazzy-usb-cam
