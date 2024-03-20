Follow the steps here to reproduce the motion capture (MoCap) experiments from the 2024 paper.

# Get the MoCap data
First, download the MoCap .mcap data files from the project page at Texas Data Repository: https://doi.org/10.18738/T8/OHI7HO

Ultimately, the directory structure and files be organized as follows:
```
marmot
- data
  - mocap_cam_1
    - metadata.yaml
    - rosbag2_2024_02_20-12_19_52_0.mcap
  - mocap_cam_2
    - metadata.yaml
    - rosbag2_2024_02_20-12_20_50_0.mcap
  - mocap_cam_3
    - metadata.yaml
    - rosbag2_2024_02_20-12_24_21_0.mcap
  - mocap_robot_moving_1
    - metadata.yaml
    - rosbag2_2024_02_19-12_35_10_0.mcap
  - mocap_robot_moving_2
    - metadata.yaml
    - rosbag2_2024_02_19-12_37_20_0.mcap
  - mocap_robot_moving_3
    - metadata.yaml
    - rosbag2_2024_02_19-12_38_59_0.mcap
  - mocap_robot_moving_4
    - metadata.yaml
    - rosbag2_2024_02_19-12_41_24_0.mcap
  - mocap_robot_static_1
    - metadata.yaml
    - rosbag2_2024_02_19-12_23_16_0.mcap
  - mocap_robot_static_2
    - metadata.yaml
    - rosbag2_2024_02_19-12_24_18_0.mcap
  - mocap_robot_static_3
    - metadata.yaml
    - rosbag2_2024_02_19-12_28_10_0.mcap
  - mocap_robot_static_4
    - metadata.yaml
    - rosbag2_2024_02_19-12_32_15_0.mcap
```
# Run the experiments
Navigate to the project ROS2 workspace and run each experiment file.
```
cd ~/tracking_ws
source /opt/ros/iron/setup.bash
source install/setup.bash
conda activate marmot
ros2 launch marmot run_mocap_cam_noheadset_exp.launch.py
ros2 launch marmot run_mocap_cam_headset_exp.launch.py
ros2 launch marmot run_mocap_robot_noheadset_exp.launch.py
ros2 launch marmot run_mocap_robot_headset_exp.launch.py

```
# Analyze the results
# Analyze the results
To generate the plots, compute times and accuracy values used in the paper, use the included `jupyter` notebook:
```
cd ~/tracking_ws/src/marmot
mamba activate marmot
jupyter-notebook
```
Then, open the notebook at `scripts/analysis/mocap_analysis.ipynb` and run each cell to recreate the experiment results and figures.
