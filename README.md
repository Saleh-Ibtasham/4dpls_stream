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
colcon build
source $PATH_TO_THIS_REPO/ros2_stream/install/setup.bash
```

The ros2_stream can also be sourced at the start on launching any terminal by adding a launch command to the `~/.bashrc` file. Add the line below to the `.bashrc` file:

```bash
source $PATH_TO_THIS_REPO/ros2_stream/install/setup.bash
```

#### Running the LiDAR Stream and Inferencing the Data

Open a terminal and launch the ouster-ros driver

```bash
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=$SENSOR_HOSTNAME
```

Open another terminal and cd into this repository. Run the bash-commands below:

```bash
cd $THIS_REPOSITORY
ros2 run lidar_controller stream_controller_node
```
The LiDAR data stream should now be saved in the `4D-PLS/streams/sequences` folder. Each run generates a new sequence in the folder. The folder structure would be like below:

```bash
streams/
└── semantic-kitti.yaml
└── sequences/
    └── 00/
        └── poses.txt
        └── calib.txt
         └── velodyne
            ├── 000000.bin
            ...
```

The `semantic-kitti.yaml` should be available in the same folder of this repo.

After the stream has started a sequence in the `streams/sequences` should be created and it would also be populated with valodyne point cloud files in binary format with appropriate calibration and poses.

At this point, navigate to the `4D-PLS` folder and run the following commands:

```bash
cd 4D-PLS
python test_stream.py
```
The 4D-panoptic segmentations would now be created with the 4D-PLS model and saved in the `test` folder with an appropriate logged subfolder.
The structure of the folder is as follows:

```bash
test/
└── log_#ID/
    └── predictions/
        ├── 00_0000000.ply
        ...
    └── probs/
        ├── 00_0000000.npy
        ├── 00_0000000_c.npy
        ├── 00_0000000_i.npy
        ...
    └── reports/
```
You can stop the inference by pressing <b>CTRL+C</b>.

The panoptic segmentations would be saved in the `test/probs/` folder with corresponding class and instance files. The files can be used to visualize with long 4D volumes of tracks. 

TO generate the tracks, run the following command:

```bash
python stitch_tracklets.py --predictions test/model_dir --n_test_frames $NUMBER_OF_FRAMES
```
This code will generate predictions in the format of SemanticKITTI under `test/model_dir/stitch$NUMBER_OF_FRAMES`.

#### Visualization of the Inferenced Data

Follow the instructions of the <a href="https://github.com/PRBonn/semantic-kitti-api/commit/439877833a0ca87c79643aef4c3561a1cb3a96bc">semantic-kitt-api</a> repository. Install the dependencies. A virtural environment either with <b>venv or conda</b> can be used for running this repository.

To visualize the data, use the `visualize.py` script. It will open an interactive opengl visualization of the pointclouds along with a spherical projection of each scan into a 64 x 1024 image. To run the visualization follow the command below:
```bash
python visualize.py --sequence $SEQUENCE_NO_OF_DATA --dataset relative/path/to/stream/dataset/ --predictions relative/path/to/test/model_dir/stitch$NUMBER_OF_FRAMES
```
Navigation:
- `n` is next scan,
- `b` is previous scan,
- `esc` or `q` exits.

### Authors

- Wangkang Jin
- Md Saleh Ibtasham

