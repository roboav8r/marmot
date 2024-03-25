#!/bin/bash
set -e

echo "ROS1 bridge entrypoint"

# Source ROS2 and bridge environments
source /opt/ros/$ROS1_DISTRO/setup.bash
source /opt/ros/$ROS2_DISTRO/setup.bash
ros2 daemon stop
ros2 daemon start
cd /bridge_ws
source install/setup.bash

# Display bridged messages and launch the bridge
ros2 run ros1_bridge dynamic_bridge --print-pairs
ros2 run ros1_bridge dynamic_bridge

exec "$@"