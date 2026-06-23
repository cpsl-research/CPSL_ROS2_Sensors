# Parameterized Dockerfile for CPU (ubuntu:24.04) and GPU (nvidia/cuda:13.3.0-cudnn-devel-ubuntu24.04)
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV PATH="/root/.local/bin:$PATH"

# Prevent services from starting during installation
RUN echo '#!/bin/sh\nexit 101' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d

# Install basic tools and set locale
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    git \
    sudo \
    vim \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Setup ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop & ros-dev-tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Poetry
RUN curl -sSL https://install.python-poetry.org | python3 - && \
    poetry config virtualenvs.options.system-site-packages true

# Initialize and update rosdep
RUN rosdep init && rosdep update

# Set workdir
WORKDIR /workspace/CPSL_ROS2_Sensors

# Clone repo and check out specified commit (default to e95438e00456904f7450786ea49bc83a87caa22c)
ARG REPO_URL=https://github.com/cpsl-research/CPSL_ROS2_Sensors
ARG COMMIT_HASH=e95438e00456904f7450786ea49bc83a87caa22c

RUN git clone ${REPO_URL} . && \
    git checkout ${COMMIT_HASH}

# Setup submodules and sensor dependencies depending on SENSORS build arg
ARG SENSORS=all

# Install packages needed for selected sensors
RUN apt-get update && \
    S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    # ── Radar ───────────────────────
    if echo ",$S_LIST," | grep -q ",radar,"; then \
        echo "Initializing radar submodules..."; \
        git submodule update --init --recursive src/CPSL_TI_Radar_ROS2; \
    fi && \
    # ── Livox ───────────────────────
    if echo ",$S_LIST," | grep -q ",livox,"; then \
        echo "Initializing livox submodules..."; \
        git submodule update --init --recursive src/CPSL_ROS_livox_ros_driver2; \
        echo "Installing Livox dependencies..."; \
        apt-get install -y gcc-9 g++-9 cmake; \
        git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 && \
        cd /opt/Livox-SDK2 && mkdir build && cd build && \
        cmake .. -DCMAKE_C_COMPILER=gcc-9 -DCMAKE_CXX_COMPILER=g++-9 && \
        make -j$(nproc) && make install && cd /workspace/CPSL_ROS2_Sensors; \
        bash src/CPSL_ROS_livox_ros_driver2/build_CPSL_ROS2_Sensors.sh jazzy; \
    fi && \
    # ── Ouster ──────────────────────
    if echo ",$S_LIST," | grep -q ",ouster,"; then \
        echo "Initializing ouster submodules..."; \
        git submodule update --init --recursive src/ouster-ros; \
        echo "Installing Ouster dependencies..."; \
        apt-get install -y \
            ros-jazzy-pcl-ros \
            ros-jazzy-tf2-eigen \
            ros-jazzy-rviz2 \
            build-essential \
            libeigen3-dev \
            libjsoncpp-dev \
            libspdlog-dev \
            libcurl4-openssl-dev \
            libpcap-dev \
            ros-jazzy-rmw-cyclonedds-cpp; \
    fi && \
    # ── Realsense ───────────────────
    if echo ",$S_LIST," | grep -q ",realsense,"; then \
        echo "Installing Realsense dependencies..."; \
        apt-get install -y ros-jazzy-librealsense2* ros-jazzy-realsense2-*; \
    fi && \
    # ── Leapmotion ──────────────────
    if echo ",$S_LIST," | grep -q ",leapmotion,"; then \
        echo "Initializing leapmotion submodules..."; \
        git submodule update --init --recursive src/CPSL_ROS2_LeapMotion; \
        git submodule update --init --recursive submodules/leapc-python-bindings; \
        echo "Installing Leapmotion dependencies..."; \
        if curl -fsSL --connect-timeout 5 https://repo.ultraleap.com/apt/public.key > /tmp/ultraleap.key 2>/dev/null; then \
            gpg --dearmor -o /usr/share/keyrings/ultraleap-archive-keyring.gpg < /tmp/ultraleap.key && \
            echo "deb [signed-by=/usr/share/keyrings/ultraleap-archive-keyring.gpg] https://repo.ultraleap.com/apt stable main" | tee /etc/apt/sources.list.d/ultraleap.list > /dev/null && \
            apt-get update && apt-get install -y ultraleap-hand-tracking; \
        else \
            echo "WARNING: repo.ultraleap.com is unreachable. Skipping Leapmotion SDK installation."; \
        fi; \
    fi && \
    # ── Vicon ───────────────────────
    if echo ",$S_LIST," | grep -q ",vicon,"; then \
        echo "Initializing vicon submodules..."; \
        git submodule update --init --recursive src/ros2-vicon-bridge; \
        echo "Installing Vicon dependencies..."; \
        apt-get install -y libboost-thread-dev libboost-date-time-dev ros-jazzy-diagnostic-updater; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# Run poetry setup inside workspace
RUN poetry env use /usr/bin/python3.12 && \
    poetry install

# Extra build step for leapmotion python bindings if selected
RUN S_LIST="${SENSORS}" && \
    if [ "$S_LIST" = "all" ]; then S_LIST="radar,livox,ouster,realsense,leapmotion,vicon"; fi && \
    if echo ",$S_LIST," | grep -q ",leapmotion,"; then \
        if [ ! -f /usr/share/keyrings/ultraleap-archive-keyring.gpg ]; then \
            echo "WARNING: Leapmotion SDK was not installed. Skipping cffi build."; \
        else \
            poetry install --with leapmotion && \
            cd submodules/leapc-python-bindings && \
            poetry run python build_cffi.py; \
        fi; \
    fi

# Run rosdep install (skip raw_radar_msgs)
RUN apt-get update && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src -y --rosdistro=jazzy --skip-keys "raw_radar_msgs" && \
    rm -rf /var/lib/apt/lists/*

# Colcon build
RUN . /opt/ros/jazzy/setup.sh && \
    eval "$(poetry env activate)" && \
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
COPY docker_entrypoint.sh /workspace/docker_entrypoint.sh
RUN chmod +x /workspace/docker_entrypoint.sh

# Set the default entrypoint
ENTRYPOINT ["/workspace/docker_entrypoint.sh"]
CMD ["bash"]
