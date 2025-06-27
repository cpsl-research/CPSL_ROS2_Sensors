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
    colcon build --packages-select raw_radar_msgs
    source install/setup.bash
    rosdep install --from-paths src -y --rosdistro=jazzy
    ```
    Note: we build this package for ROS2 jazzy, if you are using a different ROS distribution, some of the above packages may have changed

    - Next,configure the livox lidar ROS driver correctly by running the following commands. Note: this step only needs to be done when first installing the CPSL_ROS_Sensors Module
    ```
    cd CPSL_ROS2_Sensors/src/CPSL_ROS_livox_ros_driver2
    ./build_CPSL_ROS2_Sensors.sh jazzy
    ```

    - Finally, install the remaining ROS2 dependencies for the livox_ros_driver2
    ```
    cd CPSL_ROS2_Sensors
    rosdep install --from-paths src -y --rosdistro=jazzy
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


## 1. Starting LivoxMid360 and TI-Radar's simultaneously (UGV)

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
- If the problem persists, try running the following command to give access to the serial port:
    ```
    sudo chmod 666 /dev/ttyACM0 # or whatever port you need to specify
    ```

3. **Radar .json file** Next, setup the radar_0_IWR1843_nav.json and radar_1_IWR1843_nav.json configuration files in the [/src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs](./src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs). Note, that this only need to be performed the first time you use the radar on your system. 

4. **Configure the LivoxMid360**:  [FIRST TIME ONLY] - the first time you utilize the livox Mid360 lidar, you must set the ip address of your system to have a static ipv4 address of ```192.168.1.XX``` and a netmask of ```255.255.255.0```. Then replace the last two digits of the IP address (XX) of the lidar with the last two digits of the serial number (located on the side of the LiDAR, under the QR code). Note, if the last two digits start with a 0 (e.g.; 09), the ip address should just be the last digit (e.g.; 9). Update the ``cmd_data_ip``, ``push_msg_ip``, ``point_data_ip``, and ``imu_data_ip`` in the JSON file located at `~/CPSL_ROS2_Sensors/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json` and at `~/CPSL_ROS2_Sensors/src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`. Finally, change the ``ip`` in ``lidar_configs`` to be 192.168.1.1XX where XX is the last two digits of the IP address

5. **Build CPSL_ROS2_Sensors**
Next, build and source the CPSL_ROS2_Sensors package
```
cd CPSL_ROS2_Sensors
colcon build --symlink-install
source install/setup.bash
```


5. **Launch the sensors**
Finally, launch all of the sensors with the given bringup file

```
cd CPSL_ROS2_Sensors
source install/setup.bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true camera_enable:=false radar_enable:=false platform_description_enable:=true rviz:=false namespace:=cpsl_ugv_1
```
The parameters that can be used here are as follows: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `camera_enable`| true | on True, starts the camera node
| `lidar_enable`| true | on True, starts the livox lidar node
| `lidar_scan_enable`| false | on True, publishes a laserscan version of the livox's PC2 topic on /livox/lidar
| `radar_enable`| true | On True, launch the (front and back) TI radars
| `platform_description_enable`| true | On true, publishes the UGV robot description tf tree
| `rviz`| true | On True, displays an RViz window of sensor data

To integrate RealSense depth, use the human_movement_sensor_bringup.launch.py file instead:
ros2 launch cpsl_ros2_sensors_bringup human_movement_sensor_bringup.launch.py lidar_enable:=true lidar_scan_enable:=true camera_enable:=false radar_enable:=false realsense_enable:=true platform_description_enable:=true rviz:=false namespace:=cpsl_ugv_1
```
The parameters that can be used here are as follows: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `camera_enable`| true | on True, starts the camera node
| `lidar_enable`| true | on True, starts the livox lidar node
| `lidar_scan_enable`| false | on True, publishes a laserscan version of the livox's PC2 topic on /livox/lidar
| `radar_enable`| true | On True, launch the (front and back) TI radars
| `realsense_enable`| false | On True, launch the RealSense camera
| `platform_description_enable`| true | On true, publishes the UGV robot description tf tree
| `rviz`| true | On True, displays an RViz window of sensor data


## 2.Recording a Dataset (UGV)
Once a dataset has been captured the ```dataset_generator``` package can be used to capture datasets of time synchronized datasets from all of the sensors. The data that is currently available is as follows:

| **Data** | **Format** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `radar_pc`   | Nx4 np array with [x,y,z,vel] points  | radar point cloud data | 
| `radar_adc`   |  rx_antennas x samples_per_chirp x chirps_per_frame np array of complex int16s | adc data cubes | 
| `lidar`| Nx4 aray with [x,y,z,intensity] | full 3D lidar point cloud |
| `camera`| .png | image recorded from a camera |
| `imu_data`| Nx7 array with [time,w_x,w_y,w_z,a_x,a_y,a_z] |imu measurements (recorded at higher rate) |
| `vicon`| Nx7 array with [trans_x, trans_y, trans_z, quat_w, quat_x, quat_y, quat_z] |vicon poses for an object (recorded for each object) |
| `vehicle_odom`| Nx14 array with [time,x,y,z,quat_w,quat_x,quat_y,quat_z,vx,vy,vz,wx,wy,wz] |vehicle odometry measurements (recorded at higher rate) |
| `vehicle_vel`| Nx3 array with [time,vx,wz] |vehicle velocity measurements (recorded at higher rate) |

1. **Define .yaml file** To record a dataset, first update or create a new .yaml configuration file in `~/CPSL_ROS2_Sensors/src/dataset_generator/configs`. An example configuration file is shown below. Note, when specifying the topics, you should not put a "/" (e.g.; "/topic_name") as this will specify an absolute path and prevent dynamic namespacing.
```
dataset_generator:
  ros__parameters:
    radar_enable: True
    lidar_enable: True
    lidar_topic: "livox/lidar"
    camera_enable: False
    camera_topic: usb_cam/image_raw
    depth_enable: True
    depth_topic: "camera/cpsl_realsense/depth/image_rect_raw"
    imu_enable: True
    imu_topic: "imu"
    vicon_enable: False
    vehicle_odom_enable: True
    vehicle_odom_topic: "odom"
    base_frame: "cpsl_ugv_1/base_link"
    frame_rate_save_data: 5.0
    frame_rate_high_speed_sensors: 20.0
    dataset_path: "/home/cpsl/Downloads/datasets/CPSL_TEST_1"
```
2. **Launch the dataset_generator** : Once the .yaml configuration file has been generatoed, use the following code to launch dataset generation:
```
cd CPSL_ROS2_Sensors
colcon build --symlink-install
source install/setup.bash
ros2 launch dataset_generator record_dataset.launch.py
```

The parameters that can be used by using the ```parameter:=value``` notation: 
| **Parameter** | **Default** | **Description** |  
|-----------|--------------------------|---------------------------------------------|  
| `namespace`   | ''  | the namespace of the robot |  
| `param_file`| 'ugv_dataset.yaml' | the .yaml config file in the configs directory of the dataset_generator package.
