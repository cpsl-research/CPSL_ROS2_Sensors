# CPSL_ROS_Sensors

Set of ROS packages for various sensors developed and utilized by Duke's CPSL laboratory. This repository contains ROS1 packages for the following sensing modalitites:
* Livox Lidars
* Vicon motion capture system (to be added soon)
* TI-IWRXXX and DCA1000 radar development boards

## Installation [Built on ROS2 Jazzy, Ubuntu 24.04]:

### 1. Install ROS
1. Follow the instructions on the [ROS2 installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) website to install ROS2. If you are unfamiliar with ROS, its worth taking some of the [ROS2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html). We use ROS jazzy developed for Ubuntu 24.04. Using other ROS versions may require some changes

### 2. Install pre-requisites for CPSL_TI_Radar module

This repo additionally includes the CPSL_TI_Radar_ROS and CPSL_TI_Radar repositories for integrating with the TI IWR (and DCA1000) radar hardware. To ensure that all of the pre-requisites are installed for these code bases, perform the following steps:
1. Install all of the required C++ pre-requisites by following the "Pre-requisite packages" instructions in the "Pre-requisite packages" instructions in the [CPSL_TI_Radar_cpp github installation instructions](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp).

### 3. Install Livox-SDK

In order to use the Livox Lidar ROS nodes, you will also need to install the Livox-SDK. To do so, follow these instructions:
1. Install gcc 9.4.0 to allow for correct compilation:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update
sudo apt install gcc-9 g++-9 -y
```
2. Clone the Livox-SDK2.git repository
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```
3. Finally, build and install the Libox-SDK, making sure to specify the use of gcc 9.4.0
```
cd ./Livox-SDK2/
mkdir build
cd build
cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 ..
make
sudo make install
```
## Installing CPSL_ROS2_Sensors as ROS2 package

1. To install the CPSL_ROS2_Sensors ROS2 workspace perform the following commands
```
git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_Sensors
```

If you forgot to perform the --recurse-submodules when cloning the repository, you can use the following command to load the necessary submodules
```
cd CPSL_ROS2_Sensors
git submodule update --init --recursive
```

2. Next, install all of the required ros dependencies using the following commands.

    - First, install all other dependencies required by the CPSL_ROS2_Sensors package
    ```
    cd CPSL_ROS2_Sensors
    rosdep install --from-paths src -y --rosdistro=jazzy
    ```
    Note: we build this package for ROS2 jazzy, if you are using a different ROS distribution, some of the above packages may have changed

    - Next,configure the livox lidar ROS driver correctly by running the following commands. Note: this step only needs to be done when first installing the CPSL_ROS_Sensors Module
    ```
    cd CPSL_ROS2_Sensors/src/CPSL_ROS_livox_ros_driver2
    ./build_CPSL_ROS2_Sensors.sh jazzy
    ```

3. Next, build the ROS nodes in your catkin workspace using the following commands:
```
cd CPSL_ROS2_Sensors
colcon build --symlink-install
```

4. Finally, source the setup.bash file so that ROS can find the nodes and the messages
```
source install/setup.bash
```

# Tutorials


## 1. Starting LivoxMid360 and TI-Radar's simultaneously

1. **Radar Setup**. See the insructions in [CPSL_TI_Radar_ROS2](./src/CPSL_TI_Radar_ROS2/README.md) to ensure that each radar is setup correctly. For this code bases, each Radar should have the  mmWaveSDK3.6 demo installed.

2. **Radar connection setup** Follow these steps to connect the radars in the correct order.

- Connect the front radar, then check to confirm that the first radar connects successfully
    ```
    ls /dev/ttyACM* #should return /ttyACM0 /ttyACM1
    ```
- If it connects successfully, then connect the back radar, then check to confirm that the second radar connects successfully
    ```
    ls /dev/ttyACM* #should return /ttyACM0 /ttyACM1 /ttyACM2 /ttyACM3
    ```
- If you have trouble connecting the radar successfully, or the radars fail to connect in later steps, try running the following commands (you may have to login and then log out for changes to apply)
    ```
    sudo usermod -a -G dialout $USER
    ```

3. **Radar .json file** Next, setup the radar_0_IWR1843_nav.json and radar_1_IWR1843_nav.json configuration files in the [/src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs](./src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs). Note, that this only need to be performed the first time you use the radar on your system. 

4. **Configure the LivoxMid360**:  [FIRST TIME ONLY] - the first time you utilize the livox Mid360 lidar, you must set the ip address of your system to have a static ipv4 address of ```192.168.1.50``` and a netmask of ```255.255.255.0```. Then replace the last two digits of the IP address of the lidar with the last two digits of the serial number (located on the side of the LiDAR, under the QR code). Update this in the JSON file located at `~/CPSL_ROS2_Sensors/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json` and at `~/CPSL_ROS2_Sensors/src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`


5. Finally, run the bringup 

```
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py namespace:=/cpslCreate3 lidar_enable:=false lidar_scan_enable:=true radar_enable:=false platform_description_enable:=false rviz:=false
```

## 1. Using a Livox Mid360 lidar
1. [FIRST TIME ONLY] - the first time you utilize the livox Mid360 lidar, you must set the ip address of your system to have a static ipv4 address of ```192.168.1.50``` and a netmask of ```255.255.255.0```. Then replace the last two digits of the IP address of the lidar with the last two digits of the serial number (located on the side of the LiDAR, under the QR code). Update this in the JSON file located at `~/CPSL_ROS2_Sensors/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json` and at `~/CPSL_ROS2_Sensors/src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`

2. To obtain measurements from the lidar, use the following command. The will publish a [sensor/msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html) message on the topic "/livox/lidar".
```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py rviz_enable:=true
```

## To be updated 
Everything below this point needs to be updated

## 3.Lidar Odometry/Localization using livox lidar sensor

### Setup
In addition to installing the above package, localizing the lidar using the livox sensor also requires the map_serve package. We have used the ROS1 navigation package which can be accessed at [CPSL_ROS_Navigation_and_Mapping](https://github.com/cpsl-research/CPSL_ROS_Navigation_and_Mapping) and using the following terminal commands:
```
cd ~/catkin_ws/src/
git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS_Navigation_and_Mapping.git
rosdep install --from-paths ~/catkin_ws/src/CPSL_ROS_Navigation_and_Mapping/ --ignore-src -y --rosdistro=noetic
```

### Configure Lidar Localization Pipeline
Lidar locatlization is performed using the [cpsl_ros_lidar_odometry](./CPSL_ROS_Lidar_Odometry/) ROS package. The odometry is then launched by using the [lidar_odometry.launch](./CPSL_ROS_Lidar_Odometry/launch/lidar_odometry.launch) file. To configure the lidar odometry, you must provide the launch file with a .json odometry file located in the [configs_odometry](./CPSL_ROS_Lidar_Odometry/configs_odometry/) folder. The key parameters for the configuration file are as follows:
* **map/load_method**: the localizatino pipeline can either manually load a file or use a map server. In this tutorial we use a map server. Be sure to set the "load_method" to "server"
* **odometry_config/start_pose/**: Here, specify the approximate position (translation and heading) of the lidar in the global map. While this doesn't have to be exact (the localization pipeline can handle small errors), it needs to be quite accurate. This may take a little bit of time to get right when starting in a new environment.
* **subscriptions/point_cloud**: this is the topic that the pipeline will listen to for the lidar's point cloud. In this case, set it to "/livox/lidar" to use the livox MID360's lidar point cloud
* **/publishers/point_cloud**: The localization pipeline currently publishes a 2D slice of the lidar's 3D point cloud. By modifying the  frame_id and topic_name parameter's here, you specify the tf frame_id and topic name that the re-published point cloud will be published on.
* **/publishers/odometry_pose**: Since the pipeline was initially developed for localization, there is an "odometry frame" which is used to describe the position of the lidar sensor within the global map. By setting the topic_name, frame_id, and child_frame_id, you can specify the name of the ROS topic used to publish a tf tree transformation between the "odometry" frame and the "child" frame (i.e., the lidar's base frame). Make sure that the child_frame_id matches the point_cloud/frame_id so that the tf frame lines up
* **/publishers/tf_tree** Finally, set the map_frame_id, odom_frame_id, and base_frame_id parameters here to set up the full tf tree. the odom_frame_id should be the same as the odometry_pose/frame_id and the base_frame_id should be the same as the point_cloud/frame_id. The map_frame_id should be the name of the global map's frame id (usually just "map"). 

### Running the pipeline (with velodyne lidar)
To run the lidar's localization pipeline, perform the following commands. Note that you must run this code in multiple terminals. We recommend using [tmux](https://github.com/tmux/tmux/wiki/Installing) or a similar multi-terminal viewer

1. Start by running the velodyne ROS driver. We've disabled rviz here, but to see the raw point cloud anyways (before localization), set rviz_enable to true.

```
cd ~/[catkin_ws]/
catkin_make
source devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

2. Next, in a new terminal window (with the 1st terminal still running), start the map server. Below, replace [full_path_to_map].yaml with the full path to the global map you wish to use (e.g.; /home/cpsl/data/maps/wilkinson.yaml). For more resources, check out the tutorials for the [CPSL_ROS_Navigation_and_Mapping](https://github.com/cpsl-research/CPSL_ROS_Navigation_and_Mapping)
```
cd ~/[catkin_ws]/
source devel/setup.bash
roslaunch map_server map_server_sensor_updates.launch map_path:=/home/locobot/data/maps/cpsl_full.yaml
```

3. Finally, in a 3rd terminal window, run the terminal command. You may need to change lidar-odometry_wilkinson.json to be your custom configuration if you created one during the setup.
```
cd ~/[catkin_ws]/
source devel/setup.bash
roslaunch cpsl_ros_lidar_odometry lidar_odometry.launch config_file:=lidar_odometry_cpsl.json verbose:=false rviz:=true
```

## 4. Collecting Datasets with the TI DCA1000

If you want to collect a dataset which includes raw Radar data from a DCA1000 board, follow the instructions available at the [CPSL_TI_Radar_ROS](https://github.com/davidmhunt/CPSL_TI_Radar_ROS) github page under the tutorials section. **NOTE: ONLY THE IWR1443 + DCA1000 is supported. Future updates will support the IWR1843 and IWR6843**