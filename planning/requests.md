general feedback from testing:

1. Once completed with this task, update documentation and readme files to include the default ros2 launch files as well as setting up the udev rules. 

2. create special doccker .json files for the ti radars with the ports setup such that they use the /dev/ti ... nomenclature as well as that the ethernet ports for the dca1000 use the 0.0.0.0 ip address for the machine's static ip addresss (the fpga's is still the normla one, but the host ip address if needed is set to 0.0.0.0 so that it works with the docker containers)

3. for the docker compose files, instead of a normal and a "sim" variant. Use a normal one (which doesn't mount the repo as a volume and just directly installs it) and a new "dev" (i.e. delete sim) which mounts the repo as a volume like we've been doing so that changes are immediately reflected.