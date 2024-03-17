# MaRMOT
Modular and Reconfigurable Multiple Object Tracking (MaRMOT) framework for robots and intelligent systems in ROS2. MaRMOT is designed to be fast and accurate enough for real-time applications, modular, reconfigurable, and capable of running on embedded hardware.

MaRMOT was submitted to the 33rd IEEE International Conference on Robot and Human Interactive Communication (IEEE RO-MAN 2024).

This package is under active development as part of my Ph.D. in robotics at UT Austin--if there is a feature you would like to see, please contact me or raise an issue!

![](media/MaRMOT.png)

# About
MaRMOT is an open-source ROS2 implementation of the Tracking-by-Detection paradigm, used by many common trackers with deep learning-based LiDAR and vision detectors. However, MaRMOT can be extended for an arbitrary detector type.

# Usage

## Installation and Setup
To install MaRMOT, follow the [installation instructions](docs/INSTALL.md).

## Running nuScenes experiments
During development, I used the nuScenes tracking development kit for tuning and evaluation, which were reported in our 2024 RO-MAN paper.

If interested in using nuScenes for tracker development, see the [nuScenes instructions](docs/NUSCENES.md).

## Running MoCap experiments

# Development
## Adding new process models
To add a new process model, 

- In `datatypes.py`: Add an appropriate entry eithin the `Track.__init__`, `Track.predict`, and `Track.compute_proc_model` definitions. 
- In `output.py`:
- In `tracker.py`: Update `supported_proc_models`; add a `['obs_model']['proc_model']` entry for the process model; ensure `declare_obj_params` and `set_obj_properties` have entries for the necessary parameters;

# Acknowledgements
The nuScenes `.mcap` conversion script is a modified version of the original from Foxglove, available [here](https://github.com/foxglove/nuscenes2mcap). While the original Foxglove version uses protobuf serialization, the [included file](scripts/nuscenes/nuscenes_to_mcap.py) uses Foxglove's ROS2 serialization, with the same datatypes. 

The [tracking evaluation script](scripts/evaluate.py) is copied from Nutonomy's nuScenes devkit repo [here](https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/nuscenes/eval/tracking) for convenience.

# Potential Improvements
## Usability
- Docker/containerized version
## Readability
- Move detector initialization to separate class
- Move process models to separate class
- Move yaw correction to separate utility class
