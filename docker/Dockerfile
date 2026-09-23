# Parameterized Dockerfile for CPU (ubuntu:24.04) and GPU (nvidia/cuda:13.3.0-cudnn-devel-ubuntu24.04)
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Prevent services from starting during installation
RUN echo '#!/bin/sh\nexit 101' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d

# Install basic tools and set locale
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    ca-certificates \
    gnupg2 \
    lsb-release \
    git \
    sudo \
    vim \
    tmux \
    python3-pip \
    iputils-ping \
    iproute2 \
    fping \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Setup ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 variant. ros-base is the default because this image runs headless on a
# robot: ros-jazzy-desktop was measured at 203.7 s and the bulk of a 9.35 GB
# image, almost all of it rviz2 and the GUI stack that a robot never displays.
# Pass ROS_PACKAGES=ros-jazzy-desktop for a workstation image. Note that the
# ouster block below installs ros-jazzy-rviz2 explicitly when it needs it.
ARG ROS_PACKAGES="ros-jazzy-ros-base"
RUN apt-get update && apt-get install -y \
    ${ROS_PACKAGES} \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# uv from its published image rather than an install script. The venv lives
# OUTSIDE the source tree: the compose files bind-mount the host workspace over
# /workspace/CPSL_ROS2_Sensors, and a venv in there would be shadowed by (or
# collide with) the host's own .venv.
COPY --from=ghcr.io/astral-sh/uv:0.12.16 /uv /usr/local/bin/uv
ENV UV_PROJECT_ENVIRONMENT=/opt/cpsl-sensors-venv \
    UV_LINK_MODE=copy

# Initialize and update rosdep (ensuring apt update runs first and failing fast if update fails)
RUN apt-get update && \
    (rosdep init || true) && \
    rosdep update --rosdistro=jazzy || true && \
    rm -rf /var/lib/apt/lists/*

# Set workdir
WORKDIR /workspace/CPSL_ROS2_Sensors

# Everything from here down to the COPY below is a DEPENDENCY layer: none of it
# reads the workspace source, so editing source no longer invalidates the
# Livox-SDK2 compile and the apt installs. The source copy happens afterwards.
ARG SENSORS=all

# Save the enabled sensors list to a system file so that build scripts can query it
RUN echo "${SENSORS}" > /etc/cpsl_sensors_list

# Per-driver system dependencies, one script each under scripts/deps/ — the
# same scripts install.sh runs on a host and console runs when a driver is
# enabled from its GUI, so the three can no longer drift apart. Only the deps
# scripts are copied here, so editing source does not invalidate this layer.
# Leap Motion's SDK comes from a repo that is not always reachable; as before,
# the image still builds without it and says so.
COPY scripts/deps/ /workspace/CPSL_ROS2_Sensors/scripts/deps/
RUN S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    for s in $(echo "$S_LIST" | tr ',' ' '); do \
        echo "=== deps: $s ==="; \
        if [ "$s" = "leapmotion" ]; then \
            bash scripts/deps/leapmotion.sh || echo "WARNING: Leap Motion SDK not installed."; \
        else \
            bash scripts/deps/$s.sh; \
        fi; \
    done && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies from the manifests only. [tool.uv] package = false, so uv
# never builds the root project and does not need the source tree.
COPY pyproject.toml uv.lock README.md ./
COPY scripts/setup_venv.sh ./scripts/setup_venv.sh
RUN bash scripts/setup_venv.sh

# ---- source copy. Everything below re-runs when source changes. ----
COPY . /workspace/CPSL_ROS2_Sensors

# Disable unused sensor packages by adding COLCON_IGNORE
RUN S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    if ! echo ",$S_LIST," | grep -q ",radar,"; then touch src/CPSL_TI_Radar_ROS2/COLCON_IGNORE; fi && \
    if ! echo ",$S_LIST," | grep -q ",livox,"; then touch src/CPSL_ROS_livox_ros_driver2/COLCON_IGNORE; fi && \
    if ! echo ",$S_LIST," | grep -q ",ouster,"; then touch src/ouster-ros/COLCON_IGNORE; fi && \
    if ! echo ",$S_LIST," | grep -q ",leapmotion,"; then touch src/CPSL_ROS2_LeapMotion/COLCON_IGNORE; fi && \
    if ! echo ",$S_LIST," | grep -q ",vicon,"; then touch src/ros2-vicon-bridge/COLCON_IGNORE; fi

# livox_ros_driver2 ships package_ROS1.xml / package_ROS2.xml and a script that
# picks one; without it colcon fails with "package.xml does not exist", which
# reads like a corrupt checkout rather than a missing prep step.
RUN S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    if echo ",$S_LIST," | grep -q ",livox,"; then \
        bash src/CPSL_ROS_livox_ros_driver2/build_CPSL_ROS2_Sensors.sh jazzy; \
    fi

# Leap Motion python bindings + cffi build, into the venv, when selected and
# the SDK made it in.
RUN S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    if echo ",$S_LIST," | grep -q ",leapmotion,"; then \
        if [ ! -f /usr/share/keyrings/ultraleap-archive-keyring.gpg ]; then \
            echo "WARNING: Leapmotion SDK was not installed. Skipping cffi build."; \
        else \
            bash scripts/setup_venv.sh --leapmotion; \
        fi; \
    fi

# Run rosdep install (skip raw_radar_msgs)
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep update --rosdistro=jazzy || true && \
    rosdep install --from-paths src -y --rosdistro=jazzy --skip-keys "raw_radar_msgs" && \
    rm -rf /var/lib/apt/lists/*

# Colcon build
RUN . /opt/ros/jazzy/setup.sh && \
    . "$UV_PROJECT_ENVIRONMENT/bin/activate" && \
    S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    if echo ",$S_LIST," | grep -q ",radar,"; then \
        python3 -m colcon build --packages-select raw_radar_msgs --symlink-install; \
    fi && \
    if echo ",$S_LIST," | grep -q ",ouster,"; then \
        python3 -m colcon build --packages-select ouster_ros ouster_sensor_msgs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    fi && \
    python3 -m colcon build --base-paths src --symlink-install

# Copy docker entrypoint script
COPY docker/docker_entrypoint.sh /workspace/docker_entrypoint.sh
RUN chmod +x /workspace/docker_entrypoint.sh

# Set the default entrypoint
ENTRYPOINT ["/workspace/docker_entrypoint.sh"]
CMD ["bash"]
