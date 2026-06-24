general feedback from testing:

1. update the docker image generation to support custom tagging so that I can specify the version/names of the images more easily

2. when running rosdep init or install, i found that apt-get update needs to be run first. Also sometimes certain keys in the rosdep update will fail, so just set the output of this command to be true once it runs (if possible run in quite mode as well)

3. instead of the python script, install fping and use it to find the ip-addresses of the lidar sensor using a simple command line call (passing in the ip-address range to check as command line arguments)

4. in the tutorials (or in a simple script), add instructions for how to rebuild the CPSL_ROS2_Sensors package so that it can be rebuilt if changes are made at some point

5. add support for the iputils-ping and iproute2 packages in the docker build/installation process

6. Currently, I'm cloning to a specific commit in the docker image generation script so that I can stay on a specific commit. Instead of this, I'd like to clone the repository and its dependencies onto my current machine, then setup the docker container, and then mount the core ros packages as a shared volume between my machine and the docker container. Then once this is mounted, proceed with building and installing the package as normal

7. when setting up the networking, the host-ip from the docker container perspective should be 0.0.0.0. Add a note in the docker instructions for setting up networking and I'll go in and create custom configurations at a later date. 