#!/bin/bash 
set -e 
echo "USB cam entrypoint" 

# Source ROS2 and bridge environments 
source "/opt/ros/$ROS1_DISTRO/setup.bash" 
cd /ros1_ws 
source devel/setup.bash 

# Display bridged messages and launch the bridge 
roslaunch usb_cam usb_cam-test.launch
exec "$@"