# Prerequisites
This assumes you are using:
- Ubuntu 22.04 operating system.
- [`conda`](https://conda.io/projects/conda/en/latest/user-guide/install/linux.html) (or the equivalent [`mamba`](https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html)) for virtual environment management.
- ROS2 installed. MaRMOT was developed with ROS2 (Humble will work), installed from binary using the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#). 

Note: Make sure that you deactivate any virtual environments before installing ROS2!

# Setup
## Create workspace and clone the repo
```
cd ~
mkdir -p tracking_ws/src && cd tracking_ws/src
git clone https://github.com/roboav8r/ar_track_alvar_msgs.git -b ros2
git clone https://github.com/roboav8r/tracking_msgs.git -b ros2
git clone --depth 1 https://github.com/roboav8r/marmot.git -b ssti
cd ~/tracking_ws
rosdep install -i --from-path src --rosdistro humble -y
```
## Set up virtual environments
Note: either `mamba` or `conda` will work for these commands.
```
cd ~/tracking_ws/src/marmot
mamba env create -f marmot_env.yml
mamba env create -f marmot_eval.yml # OPTIONAL - if using nuScenes evaluation
```

## Building the package
```
mamba activate marmot
cd ~/tracking_ws
source /opt/ros/${YOUR_ROS2_DISTRO}/setup.bash
colcon build --packages-select tracking_msgs
source install/setup.bash
colcon build
source install/setup.bash
```
