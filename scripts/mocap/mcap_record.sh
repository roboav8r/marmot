#!/bin/bash

ros2 bag record -s mcap tf tf_static philbart/lidar_points vrpn_client_node/philbart_vicon/pose vrpn_client_node/groundtruth_1/pose vrpn_client_node/groundtruth_2/pose oak/nn/spatial_detections oak/rgb/camera_info oak/rgb/image_raw oak/stereo/camera_info oak/stereo/image_raw

# TODO - add OAK-D, hololens topics