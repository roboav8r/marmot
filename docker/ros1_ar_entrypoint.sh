#!/bin/bash 
set -e 
echo "AR Track entrypoint" 

# Source ROS2 and bridge environments 
source "/opt/ros/$ROS1_DISTRO/setup.bash" 
cd /ros1_ws 
source devel/setup.bash 

# Display bridged messages and launch the bridge 
roslaunch ar_track_alvar pr2_indiv_no_kinect.launch marker_size:=$MARKER_SIZE cam_image_topic:=$AR_CAM_TOPIC cam_info_topic:=$AR_INFO_TOPIC output_frame:=$OUTPUT_FRAME
exec "$@"