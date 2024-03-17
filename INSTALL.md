# Prerequisites
This assumes you are using:
- Ubuntu 22.04 operating system.
- [`conda`](https://conda.io/projects/conda/en/latest/user-guide/install/linux.html) (or the equivalent [`mamba`](https://mamba.readthedocs.io/en/latest/installation/mamba-installation.html)) for virtual environment management.
- ROS2 installed. MaRMOT was developed with ROS2 Iron, installed from binary using the instructions [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html#). 

Note: Make sure that you deactivate any virtual environments before installing ROS2!

# Setup
## Clone the repo
```
cd ~
mkdir -p tracking_ws/src && cd tracking_ws/src
git clone https://github.com/roboav8r/coda-models.git
git clone https://github.com/roboav8r/marmot.git
git clone https://github.com/roboav8r/tracking_msgs.git
cd tracking_ws
rosdep install -i --from-path src --rosdistro iron -y
```
## Setting up the Conda environments
Note: either `mamba` or `conda` will work for these commands.
```
mamba env create -f marmot_env.yml
mamba env create -f marmot_eval.yml
```

## Building the package
```
cd ~/tracking_ws
source /opt/ros/${YOUR_ROS2_DISTRO}/setup.bash
colcon build --packages-select tracking_msgs
source install/setup.bash
colcon build
source install/setup.bash
```
