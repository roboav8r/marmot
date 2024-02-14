# MaRMOT
Modular and Reconfigurable Multiple Object Tracking (MaRMOT) framework for robots and intelligent systems in ROS2. MaRMOT is designed to be fast and accurate enough for real-time applications, modular, reconfigurable, and capable of running on embedded hardware. 

This package is under active development as part of my Ph.D. in robotics at UT Austin--if there is a feature you would like to see, please contact me or raise an issue!

![](media/MaRMOT.png)

# Prerequisites
This assumes you are using Ubuntu 22.04 with ROS2 and `conda` installed. This was developed using ROS2 Iron, installed from binary using the instructions [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html#). 

Note: Make sure that you deactivate any virtual environments before installing ROS2.

# Setup
## Clone the repo
```
cd ~
mkdir -p tracking_ws/src && cd tracking_ws/src
git clone https://github.com/roboav8r/marmot.git
git clone https://github.com/roboav8r/tracking_msgs.git
cd tracking_ws
rosdep install -i --from-path src --rosdistro iron -y
```
## Setting up the Conda environment
Note: I use [`mamba`](https://github.com/conda-forge/miniforge) as a personal preference, but it is a drop-in replacement to `conda`; either `mamba` or `conda` will work for these commands.
```
mamba env create -f marmot_env.yml
mamba activate marmot
```

## Building the package
```
cd ~/tracking_ws
source /opt/ros/${YOUR_ROS2_DISTRO}/setup.bash
colcon build --packages-select tracking_msgs
source install/setup.bash
colcon build
```

## Download nuScenes data (Optional)
If you want to evaluate MaRMOT on nuScenes OR use the nuScenes dataset for tracking development & evaluation, follow these steps to download and extract the dataset.

1) [Create an account with nuScenes at this link](https://www.nuscenes.org/sign-up).
2) Create a directory to store the data and navigate to it:
```
cd ~
mkdir nuscenes
cd nuscenes
```
4) At the [downloads section of the nuScenes page](https://www.nuscenes.org/nuscenes#download), download the US versions of the Mini, Trainval, and Test Metadata and file blobs. Also, download the Map expansion v1.3.

  NOTE: If using a headless display (e.g. a server), you can use wget to download the files as described [here](https://github.com/nutonomy/nuscenes-devkit/issues/110). An example command format is provided below:
  ```
  wget -O v1.0-mini.tgz "https://s3.amazonaws.com/data.nuscenes.org/public/v1.0/v1.0-mini.tgz?..."
  ```
Upon completion, you should have the following files in your `~/nuscenes` directory:
```
nuScenes-map-expansion-v1.3.zip
v1.0-mini.tgz
v1.0-test_blobs.tgz
v1.0-test_meta.tgz
v1.0-trainval01_blobs.tgz
v1.0-trainval02_blobs.tgz
v1.0-trainval03_blobs.tgz
v1.0-trainval04_blobs.tgz
v1.0-trainval05_blobs.tgz
v1.0-trainval06_blobs.tgz
v1.0-trainval07_blobs.tgz
v1.0-trainval08_blobs.tgz
v1.0-trainval09_blobs.tgz
v1.0-trainval10_blobs.tgz
v1.0-trainval_meta.tgz
```
5) Extract the blob files. For each `.tgz` file, extract it as follows:
```
tar -xvf v1.0-mini.tgz
tar -xvf v1.0-test_blobs.tgz
tar -xvf v1.0-test_meta.tgz
tar -xvf v1.0-trainval01_blobs.tgz
...
tar -xvf v1.0-trainval_meta.tgz
```
6) Unzip the map expansion data into the `maps` folder:
```
unzip nuScenes-map-expansion-v1.3.zip -d maps
```

7) Download and unzip the megvii detector:
```
wget https://www.nuscenes.org/data/detection-megvii.zip
unzip detection-megvii.zip -d detection-megvii
```

Upon completion, the directory should have the following structure:
```
~/nuscenes
├── detection-megvii
├── detection-megvii.zip
├── maps
├── samples
├── sweeps
├── v1.0-mini
├── v1.0-test
├── v1.0-trainval
```
At this point, you can remove the .zip and .tgz files if you'd like.
# Usage

## Running nuScenes experiments
If you wish to run the nuScenes experiments OR use the nuScenes dataset for tracker development, complete the following steps:

1) Source the workspace. At a terminal:
```
mamba activate marmot
source /opt/ros/${YOUR_ROS2_DISTRO}/setup.bash
cd ~/tracking_ws
source install/setup.bash

```

2) Convert nuscenes detection and data to ROS2 .mcap files. In the `MaRMOT` directory:
```
python3 scripts/nuscenes_to_mcap.py # Converts mini_train by default
python3 scripts/nuscenes_to_mcap.py -s mini_val # Converts the mini_val split
python3 scripts/nuscenes_to_mcap.py -d v1.0-trainval -s train # Converts the training split from the main dataset
python3 scripts/nuscenes_to_mcap.py -d v1.0-trainval -s val # Converts the validation split from the main dataset
python3 scripts/nuscenes_to_mcap.py -d v1.0-test -s test # Converts the test split from the main dataset
```

3) Launch the nuScenes experiment manager node to compute tracking results. At a properly sourced workspace:
```
ros2 launch marmot run_nuscenes_exp_minival.launch.py # To compute results for nuScenes' mini validation set (ideal for quick development), or
ros2 launch marmot run_nuscenes_exp_val.launch.py # To compute results for nuScenes' full validation set
```

# Acknowledgements
The nuScenes `.mcap` conversion script is a modified version of the original from Foxglove, available [here](https://github.com/foxglove/nuscenes2mcap). While the original Foxglove version uses protobuf serialization, the [included file](scripts/nuscenes_to_mcap.py) uses Foxglove's ROS2 serialization, with the same datatypes. 

The [tracking evaluation script](scripts/evaluate.py) is copied from Nutonomy's nuScenes devkit repo [here](https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/nuscenes/eval/tracking) for convenience.

# Potential Improvements
## Usability
- Docker/containerized version
## Readability
- Move detector initialization to separate class
- Move process models to separate class
- Move yaw correction to separate utility class