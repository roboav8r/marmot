# # This is an auto generated Dockerfile for ros:ros1-bridge
# # generated from docker_images_ros2/ros1_bridge/create_ros_ros1_bridge_image.Dockerfile.em
FROM ros:galactic-ros-base-focal

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
run echo ""
run echo ""

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO galactic

# install ros1 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm \
    ros-noetic-tf \
    ros-noetic-tf2-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-diagnostic-msgs \
    ros-noetic-roscpp-tutorials=0.10.2-1* \
    ros-noetic-rospy-tutorials=0.10.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-galactic-tf2-msgs \
    ros-galactic-sensor-msgs \
    ros-galactic-diagnostic-msgs \
    ros-galactic-rviz-common \
    qtbase5-dev \
    ros-galactic-rviz-default-plugins \
    ros-$ROS2_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS2_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS2_DISTRO-rmw \ 
    && rm -rf /var/lib/apt/lists/*

# Clone bridge source code
RUN mkdir -p /bridge_ws/src
WORKDIR /bridge_ws/src
RUN git clone -b ${ROS2_DISTRO} https://github.com/ros2/ros1_bridge.git

WORKDIR /bridge_ws
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-skip ros1_bridge

# Build workspace and ros1_bridge from source
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \ 
    source /bridge_ws/install/setup.bash && \ 
    colcon build --packages-select ros1_bridge --cmake-force-configure

# setup entrypoint
COPY ./bridge_entrypoint.sh /
