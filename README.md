# CPSL_ROS2_Sensors

Set of ROS packages for various sensors developed and utilized by Duke's CPSL laboratory. This repository contains ROS1 packages for the following sensing modalitites:
* Livox Lidars
* Vicon motion capture system (to be added soon)
* TI-IWRXXX and DCA1000 radar development boards

## Installation [Built on ROS2 Jazzy, Ubuntu 24.04]:

### 1. Pre-requisites

If you haven't already done so, please perform the following steps to install the pre-requisites for this repository:

#### 1.1 Install ROS 2
Follow the instructions on the [ROS2 installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) website to install ROS2. If you are unfamiliar with ROS, it is worth taking some of the [ROS2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html). We use ROS Jazzy developed for Ubuntu 24.04. Using other ROS versions may require some changes.

#### 1.2 Install TI Radar Dependencies
This repo integrates with the CPSL_TI_Radar packages. To ensure all pre-requisites are installed:
1. Install all required C++ pre-requisites by following the "Pre-requisite packages" instructions in the [CPSL_TI_Radar_cpp github installation instructions](https://github.com/davidmhunt/CPSL_TI_Radar/tree/main/CPSL_TI_Radar_cpp).

#### 1.3 Install Livox-SDK
To use the Livox Lidar ROS nodes, install the Livox-SDK:
1. Install gcc 9.4.0:
    ```bash
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
    sudo apt update
    sudo apt install gcc-9 g++-9 -y
    ```
2. Clone the Livox-SDK2 repository:
    ```bash
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    ```
3. Build and install Livox-SDK, specifying gcc 9.4.0:
    ```bash
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 ..
    make
    sudo make install
    ```

#### 1.4 Install Intel Realsense Dependencies
To use an Intel Realsense camera (taken from [realsense-ros](https://github.com/IntelRealSense/realsense-ros)):
1. **Install Intel Realsense SDK2.0**:
    ```bash
    # REPLACE <ROS_DISTRO> WITH YOUR ROS DISTRO (e.g.; jazzy)
    sudo apt install ros-<ROS_DISTRO>-librealsense2* 
    ```
2. **Install Intel Realsense2 ROS Nodes**:
    ```bash
    # REPLACE <ROS_DISTRO> WITH YOUR ROS DISTRO (e.g.; jazzy)
    sudo apt install ros-<ROS_DISTRO>-realsense2-*
    ```

#### 1.5 Install Leap Motion Dependencies (Optional)
If you want to use the LeapMotion2 hand tracking sensor:
1. Install Ultraleap Gemini for Ubuntu 22.04/24.04:
    ```bash
    # Add the Ultraleap GPG key
    wget -qO - https://repo.ultraleap.com/keys/apt/gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ultraleap.gpg

    # Add Ultraleap repo to apt
    echo 'deb [arch=amd64] https://repo.ultraleap.com/apt stable main' | sudo tee /etc/apt/sources.list.d/ultraleap.list

    # Update apt
    sudo apt update

    # Install Ultraleap packages
    sudo apt install ultraleap-hand-tracking
    ```
    - Verify installation:
    ```bash
    ultraleap-hand-tracking-control-panel
    ```

#### 1.6 Install Python Poetry
1. Check if Poetry is installed:
    ```bash
    poetry --version
    ```
2. If not installed, install it:
    ```bash
    curl -sSL https://install.python-poetry.org | python3 -
    ```
    *Troubleshooting*: If you encounter keyring errors:
    ```bash
    export PYTHON_KEYRING_BACKEND=keyring.backends.null.Keyring
    ```

### 2. Installing CPSL_ROS2_Sensors

#### 2.1 Clone and Configure
1. Configure poetry to use system packages:
    ```bash
    poetry config virtualenvs.options.system-site-packages true
    ```
2. Clone the repository with submodules:
    ```bash
    git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_Sensors
    ```
    *If you forgot submodules*: `git submodule update --init --recursive` inside the repo.

#### 2.2 Install Python Environment
1. Setup the environment (ensure you use the system python version matching your ROS install, e.g., 3.12 for Jazzy):
    ```bash
    cd CPSL_ROS2_Sensors
    poetry env use /usr/bin/python3.12
    ```
2. Install dependencies:
    ```bash
    poetry install # standard
    # OR
    poetry install --with leapmotion # if using leap motion
    ```

3. **(Leap Motion Only)** Build bindings:
    ```bash
    eval $(poetry env activate)
    cd submodules/leapc-python-bindings
    python -m build leapc-cffi
    pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz
    ```

#### 2.3 Build ROS Packages
1. Install ROS dependencies and build:
    ```bash
    cd CPSL_ROS2_Sensors
    eval $(poetry env activate)
    
    # 1. Install ROS dependencies, skipping the local raw_radar_msgs
    rosdep install --from-paths src -y --rosdistro=jazzy --skip-keys "raw_radar_msgs"

    # 2. Configure Livox Lidar Driver (First time only)
    cd src/CPSL_ROS_livox_ros_driver2
    ./build_CPSL_ROS2_Sensors.sh jazzy
    cd ../..

    # 3. Build raw_radar_msgs first to ensure visibility
    python -m colcon build --packages-select raw_radar_msgs --symlink-install

    # 4. Build the rest of the workspace
    python -m colcon build --base-paths src --symlink-install
    ```

2. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## 3. Hardware & Sensor Setup

After installation, the RealSense and Leap Motion sensors should be ready to use. However, the Livox LIDAR and TI Radars require additional configuration.

### 3.1 Radar Setup
This repository uses the `CPSL_TI_Radar_ROS2` package to interface with TI mmWave radars.
1.  **JSON Configuration**: Radar launch files use JSON config files located in `src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs`.
    -   **Important Fields**:
        -   `CLI_port`: The serial port for the configuration UART (e.g., `/dev/ttyACM0`).
        -   `data_port`: The serial port for the data UART (e.g., `/dev/ttyACM1`).
        -   `DCA1000_streaming`: Set `enabled: true` if using a DCA1000 capture card.
        -   `TI_Radar_config_path`: Path to the `.cfg` chirp profile.
2.  **Connection Order**:
    -   **Front Radar**: Connect first. Usually appears as `/dev/ttyACM0` (CLI) and `/dev/ttyACM1` (Data).
    -   **Back Radar**: Connect second. Usually appears as `/dev/ttyACM2` (CLI) and `/dev/ttyACM3` (Data).
3.  **Permissions**:
    -   Ensure your user is in the `dialout` group to access serial ports:
        ```bash
        sudo usermod -a -G dialout $USER
        ```
        *Log out and back in for this to take effect.*

    - To confirm that this worked correctly, connect either of the radars and then run the following command:
        ```
        ls /dev/ttyACM* # for 1843/1443 should return /ttyACM0 /ttyACM1 /ttyACM2 /ttyACM3
        ls /dev/ttyUSB* #for 6843, should return /ttyUSB0 /ttyUSB1
        ```

    - If the problem persists, try running the following command to give access to the serial port:
        ```
        sudo chmod 666 /dev/ttyACM0 # or whatever port you need to specify
        ```

### 3.2 Livox LiDAR Setup (Mid360)
The Livox Mid360 requires a static IP connection.
1.  **Network Configuration**:
    -   Set your computer's wired network interface to a static IPv4 address:
        -   IP: `192.168.1.XX` (e.g., `192.168.1.50`).
            - Note: replace the last two digits of the IP address (XX) of the lidar with the last two digits of the serial number (located on the side of the LiDAR, under the QR code).
            - If the last two digits start with a 0 (e.g.; 09), the ip address should just be the last digit (e.g.; 9).
        -   Netmask: `255.255.255.0`.
2.  **Update Config File**:
    -   Locate the serial number on your LiDAR (under the QR code).
    -   Modify `src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`:
        -   Update `ip` in `lidar_configs` to `192.168.1.1XX` (where XX are the last two digits of the serial number).
        -   *Note*: If the serial ends in `09`, the IP should be `...109`.


---

## 4. Tutorials

This section details how to launch the sensor systems for data collection.

### 4.1 UGV Data Collection
These launch files bring up the `livox_lidar`, `ti_radars`, and `usb_camera` for the iRobot Create3 UGV.

| Launch File | Description | Radar Configuration |
| :--- | :--- | :--- |
| **`ugv_sensor_bringup.launch.py`** | **Standard Setup**. Uses standard velocity/range profiles. | • Front: `radar_0_IWR1843_vel_sr.json`<br>• Back: `radar_1_IWR1843_vel_sr.json` |
| **`ugv_sensor_bringup_ragnnarok.launch.py`** | **RaGNNarok Setup**. Configured for the RaGNNarok paper dataset. | • Front: `front_radar_IWR1843_RaGNNarok_UGV_5m.json`<br>• Back: `back_radar_IWR1843_RaGNNarok_UGV_5m.json` |

**Example Usage**:
```bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py \
    lidar_enable:=true \
    radar_enable:=true \
    camera_enable:=true \
    namespace:=cpsl_ugv_1
```
*Common Arguments:*
- `lidar_enable`: `true`/`false`
- `radar_enable`: `true`/`false`
- `camera_enable`: `true`/`false`
- `rviz`: `true`/`false` (Open RViz for visualization)

### 4.2 UAV Data Collection
These launch files bring up sensors for the UAV platform, including Downward-facing radar and LiDAR.

| Launch File | Description | Radar Configuration |
| :--- | :--- | :--- |
| **`uav_sensor_bringup_radsar.launch.py`** | **Standard (RadSAR)**. Setup for SAR/Velocity collection. | • Front: `front_radar_IWR1843_dca_RadVel_10Hz.json`<br>• Down: `down_radar_IWR6843_ods_dca_RadVel.json` |
| **`uav_sensor_bringup_ragnnarok.launch.py`** | **RaGNNarok**. Setup for RaGNNarok paper. | • Front: `front_radar_IWR1843_RaGNNarok_UAV_5m.json`<br>• Back: `back_radar_IWR1843_RaGNNarok_UAV_5m.json` |

**Example Usage**:
```bash
ros2 launch cpsl_ros2_sensors_bringup uav_sensor_bringup_ragnnarok.launch.py \
    front_radar_enable:=true \
    down_radar_enable:=true \
    lidar_enable:=true
```

### 4.3 Human Movement Data Collection
Captures data for human motion correlation using Leap Motion (Hands), RealSense (Depth/RGB), and Radars.

**Launch File**: `human_movement_sensor_bringup.launch.py`

**Example Usage**:
```bash
ros2 launch cpsl_ros2_sensors_bringup human_movement_sensor_bringup.launch.py \
    leapmotion_enable:=true \
    realsense_enable:=true \
    radar_enable:=true
```

### 4.4 Single Radar Testing
Bring up a single radar for testing purposes.

**Launch File**: `sensor_bringup_single_radar.launch.py`

**Example Usage**:
```bash
ros2 launch cpsl_ros2_sensors_bringup sensor_bringup_single_radar.launch.py
```

### 4.5 Recording Datasets
The `dataset_generator` package synchronizes and saves data from all active sensors.

**1. Create/Edit a Configuration YAML:**
Create a file in `src/dataset_generator/configs/` (e.g., `my_experiment.yaml`).

```yaml
dataset_generator:
  ros__parameters:
    # --- Sensor Enables ---
    radar_enable: True
    lidar_enable: True
    camera_enable: True
    depth_enable: False
    imu_enable: True
    vehicle_odom_enable: True
    
    # --- Topic Names ---
    lidar_topic: "livox/lidar"
    camera_topic: "usb_cam/image_raw"
    imu_topic: "livox/imu"
    
    # --- Storage ---
    # Path where the dataset folder will be created
    dataset_path: "/home/cpsl/Downloads/datasets/experiment_1"
    
    # --- Frames ---
    base_frame: "cpsl_ugv_1/base_link"
    
    # --- Rates ---
    frame_rate_save_data: 10.0  # Hz
```

**2. Launch the Recorder:**
```bash
ros2 launch dataset_generator record_dataset.launch.py param_file:=my_experiment.yaml
```
*Note: The `param_file` argument looks for files inside the `configs/` directory.*

