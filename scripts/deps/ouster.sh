#!/usr/bin/env bash
# Ouster OS-series lidar: build dependencies of ouster-ros, plus fping for
# scripts/find_ouster_ip.sh. Selecting cyclonedds as the RMW is shell setup and
# stays in install.sh.
source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

apt_ensure \
    fping \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-rviz2 \
    build-essential \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libpcap-dev \
    ros-jazzy-rmw-cyclonedds-cpp
