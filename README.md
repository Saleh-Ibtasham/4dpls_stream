# Master-PanopticSegmentationAugmentation-Wangkang-Saleh

This is repository includes Python3 scripts for data conversion, data augmentation and 4DPLS model during degree project "Point Cloud Data Augmentation for 4D Panoptic Segmentation". This repository also includes the process of streaming LiDAR data into the 4DPLS model for real-time inference.


### Overview of files

* __geo.py__:  Script to do geometric augmentation for point cloud data 
* __pcd2bin.py__: Script to convert .pcd format to .bin format
* __4D-PLS__: folder where the 4dpls model is built
* __4D-PLS/test_stream.py__: Script for inferencing the stream data with the model
* __ros2_stream__: Folder where the ROS2 workspace for capturing streaming data is built.

### Instructions for Streaming and Inferencing LiDAR Data

#### Create the Ouster ROS2 driver workspace

Follow the instructions of the <a href="https://github.com/ouster-lidar/ouster-ros">Ouster Ros1/Ros2 Driver</a> repository to build and connect to the lidar sensor. Use the appropriate ros2 version in your system.

However, when launching the <b>ouster_ros</b> driver change the parameter in the `sensor.launch.xml` to accept the dfault system launch qos settings, i.e., change the default value to <b>True</b>. Like below:

```xml
<arg name="use_system_default_qos" default="true"
    description="Use the default system QoS settings"/>
```

#### Create the ROS2 stream workspace

Download this repository and build the ros2_stream works space like below:

```bash
cd ros2_stream
colcon build --symlink-install
source $PATH_TO_THIS_REPO/ros2_stream/install/setup.bash
```

The ros2_stream can also be sourced at the start on launching any terminal by adding a launch command to the `~/.bashrc` file. Add the line below to the `.bashrc` file:

```bash
source $PATH_TO_THIS_REPO/ros2_stream/install/setup.bash
```

#### Pre-requisites before running the LiDAR Stream

First make sure that the computer and the LiDAR are in the same subnetwork. Run the bash-commands below to configure the network to the appropriate subnet:

```bash
ifconfig
```
This will give you a list of the network interfaces and the interface the LiDAR is connected to. In Linux 20.04 or newer the interface would typically start with `enp0s...`. This is the interface of your system which is connected to the LiDAR. To connect the system to the same subnet of the LiDAR take the ip of the LiDAR and use the first usable ip address of the network, unless the LiDAR has the first usable ip address, in that case, use any other ip address with proper subnetmask to add the interface to the proper subnet. Make sure to delete the previous subnet the system is connected to in this interface.

To remove the initial subnet the interface is connected use the following command:

```bash
sudo ip addr del 10.42.0.1/24 dev enp0s31f6
```
Assuming the ip address of the interface is `10.42.0.1` with subnetmask `/24` and the interface is `enp0s31f6`

Then simply add the desired subnet to this interface

```bash
sudo ip addr add 192.168.40.1/24 dev enp0s31f6
```
Assuming the ip address of the LiDAR is in the network `192.168.40.0/24`

#### Running the LiDAR Stream and Visualizing the Inferencing the data

Open a terminal and launch the ouster-ros driver

```bash
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=$SENSOR_HOSTNAME
```
The `$SENSOR_HOSTNAME` is the ip address of the LiDAR or the serial number provided with the LiDAR.

Open another terminal and cd into the folder `4D-PLS/`. Run the bash-commands below:

```bash
cd $THIS_REPOSITORY
ros2 run lidar_controller stream_controller_node
```
The LiDAR data stream should now be saved in the `streams/` folder.

The `semantic-kitti.yaml` should be available in the same folder of this repo.

After the stream has started a sequence in the `streams/calib.txt` and `poses.txt` should be created.

At this point, a window would open with side by side view of real-time LiDAR detection and Inference.

You can stop the inference by closing the window.

### Authors

- Wangkang Jin
- Md Saleh Ibtasham

