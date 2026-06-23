items to implement for the next plan:
1. docker container for installing and running the code. Should have the following features:
    - base images
        - for CPU machines: https://hub.docker.com/_/ubuntu with tag for ubuntu 24.04
        - for GPU machines: https://hub.docker.com/r/nvidia/cuda with 13.3.0-cudnn-devel-ubuntu24.04
    - ros2 version: https://github.com/osrf/docker_images/blob/0038f1c3a11aa0fc573d698b39ab5c204aad5a40/ros/jazzy/ubuntu/noble/ros-base/Dockerfile
        - but want full ros2 jazzy installated using something like the instructions below: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
        - also want rviz installed as well
    - networking:
        - want the ros2 container/containers to be isolated from the host (i.e. if multiple containers are spun up, I want them to be able to talk to eachother, but I don't want it to interfeere with a host machine's ros2 networking if possible. e.g. nodes in the containers shouldn't be discoverable from the host and vice versa)
        - will need to allow specific ip-addresses and ports to be passed through e.g. for lidar and radar sensors. If possible, I'd prefer that the container keep the same base host ip, but then in the docker .yaml configuration file, the host is setup such that it passes the correct base ip address and port through to the code running in the docker container
    - serial ports:
        - the serial ports must be allow to be passed through and setup in the .yaml file as well. This will be used for radar and potentially realsense/leap motion implementation as well
    - other items to install
        - install tmux and vim on the docker container as well
    - how the CPSL_ROS2_Sensors package should be installed
        - it should be installed via a git pull and then the repo should be set to a specific commit (for consistency), then a modified version of the install.sh file should be implemented/run to clone/install the necessary files and install everything as possible. call this new script install_docker.sh). 
    - testing: each part should be independently tested 
    - documentation:
        - should update the README.md file with instructions to include:
            - docker engine installation/setup
            - nvidia driver installation and setup
                e.g.: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-the-nvidia-container-toolkit and https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuration 
            - networking setup
            - usage/building the images
                - how to build the image with separate tags for cpu or gpu systems
                - how to bring up the containers with docker compose
                - how to use development containers
                - how to ensure that x11 port forwarding is setup so that things like rviz2 pass through well