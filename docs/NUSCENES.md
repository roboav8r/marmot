If you want to evaluate MaRMOT on nuScenes OR use the nuScenes dataset for tracking development & evaluation, follow these steps to download and extract the dataset.

# Download nuScenes data 
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

# Converting the nuScenes data to ROS2 .mcap format
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
python3 scripts/nuscenes/nuscenes_to_mcap.py # Converts mini_train by default
python3 scripts/nuscenes/nuscenes_to_mcap.py -s mini_val # Converts the mini_val split
python3 scripts/nuscenes/nuscenes_to_mcap.py -d v1.0-trainval -s train # Converts the training split from the main dataset
python3 scripts/nuscenes/nuscenes_to_mcap.py -d v1.0-trainval -s val # Converts the validation split from the main dataset
python3 scripts/nuscenes/nuscenes_to_mcap.py -d v1.0-test -s test # Converts the test split from the main dataset
```
# Run nuScenes experiments (general)
Launch the nuScenes experiment manager node to compute tracking results. At a properly sourced workspace:
```
ros2 launch marmot run_nuscenes_exp_minival.launch.py # To compute results for nuScenes' mini validation set (ideal for quick development), or
ros2 launch marmot run_nuscenes_exp_val.launch.py # To compute results for nuScenes' full validation set
ros2 launch marmot run_nuscenes_exp_test.launch.py # To compute results for nuScenes' test set
```
Note: To change the actual tracker configurations that are run on each split, you can edit the `config/exp_minival.yaml`, `config/exp_val.yaml`, and `config/exp_test.yaml` files' `exp_configs` lists with the config files you wish to evaluate.

# Run nuScenes experiments (2024 paper)
Launch the nuScenes experiment manager node to compute tracking results. At a properly sourced workspace:
```
mamba activate marmot
ros2 launch marmot run_nuscenes_exp_minitrain_sims.launch.py # To compute results for experiment 1 (sim metric timing)
ros2 launch marmot run_nuscenes_exp_minitrain.launch.py # To compute results for experiment 2 (matching, model, and method selection)
ros2 launch marmot run_nuscenes_exp_val.launch.py # To compute results for experiment 3a (method, matching, and models, 48 configs)
ros2 launch marmot run_nuscenes_exp_bestval.launch.py # To compute results for nuScenes' full validation set on best model, best management method, and multiple similarity metrics (3 configs)
ros2 launch marmot run_nuscenes_exp_besttest.launch.py # To compute results for nuScenes' test set for the best overall set of parameters (1 config)
```
Note: To change the actual tracker configurations that are run on each split, you can edit the `config/exp_minival.yaml`, `config/exp_val.yaml`, and `config/exp_test.yaml` files' `exp_configs` lists with the config files you wish to evaluate.

# Evaluating nuScenes results
Run the evaluation script, which recursively calls the nuScenes `evaluate.py` for all computed results.
```
cd ~/tracking_ws/src/marmot
mamba activate marmot_eval
python3 scripts/nuscenes/eval_exp_results.py -r ../results/1_sim_metrics
python3 scripts/nuscenes/eval_exp_results.py -r ../results/2_match_yc_model
python3 scripts/nuscenes/eval_exp_results.py -r ../results/3a_param_tuning
python3 scripts/nuscenes/eval_exp_results.py -r ../results/3b_best
```
Now, examine `results/evaluated_results.txt` to see nuScenes results for all the test cases!

# Analyze the results
To generate the plots and compute times and accuracy values used in the paper, use the included `jupyter` notebook:
```
cd ~/tracking_ws/src/marmot
mamba activate marmot
jupyter-notebook
```
Then, open the notebook at `scripts/analysis/nuscenes_analysis.ipynb` and run each cell to recreate the experiment results and figures.
