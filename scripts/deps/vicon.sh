#!/usr/bin/env bash
# Vicon: build dependencies of ros2-vicon-bridge.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure \
    libboost-thread-dev \
    libboost-date-time-dev \
    ros-jazzy-diagnostic-updater
