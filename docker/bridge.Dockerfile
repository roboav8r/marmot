FROM ros:galactic-ros-base-focal 
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# setup ros1 sources.list and keys 
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list 
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 

ENV ROS1_DISTRO noetic 
ENV ROS2_DISTRO galactic 

# install ros1 debian packages 
RUN apt-get update && apt-get install -y --no-install-recommends \ 
ros-noetic-audio-common \
ros-noetic-ros-comm \
ros-noetic-tf2-msgs \
ros-noetic-cmake-modules \ 
ros-noetic-cv-bridge \
ros-noetic-dynamic-reconfigure \
ros-noetic-image-transport \
ros-noetic-pcl-conversions \
ros-noetic-pcl-ros \
ros-noetic-resource-retriever \
ros-noetic-visualization-msgs \
ros-noetic-roscpp-tutorials=0.10.2-1* \
ros-noetic-rospy-tutorials=0.10.2-1* \
&& rm -rf /var/lib/apt/lists/* 

# install ros2 debian packages 
RUN apt-get update && apt-get install -y --no-install-recommends \
ros-galactic-tf2-msgs \
ros-galactic-rviz-common \
qtbase5-dev \
ros-galactic-rviz-default-plugins \
&& rm -rf /var/lib/apt/lists/* 

# Clone and build custom ROS1 messages from source 
RUN mkdir -p /ros1_ws/src 
WORKDIR /ros1_ws/src 
RUN git clone -b noetic-devel https://github.com/ros-perception/ar_track_alvar.git
WORKDIR /ros1_ws 
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
catkin_make

# Clone and build ROS2 messages from source 
RUN mkdir -p /ros2_ws/src 
WORKDIR /ros2_ws/src 
RUN git clone -b ros2 https://github.com/roboav8r/ar_track_alvar_msgs.git
WORKDIR /ros2_ws 
RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \ 
colcon build

# Clone bridge source code and build from source 
RUN mkdir -p /bridge_ws/src 
WORKDIR /bridge_ws/src 
RUN git clone -b ${ROS2_DISTRO} https://github.com/ros2/ros1_bridge.git 
WORKDIR /bridge_ws 
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
source /opt/ros/${ROS2_DISTRO}/setup.bash && \ 
colcon build --symlink-install --packages-skip ros1_bridge 

# Build workspace and ros1_bridge from source 
# ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
RUN source /ros1_ws/devel/setup.bash && \
source /ros2_ws/install/setup.bash && \
colcon build --packages-select ros1_bridge --cmake-force-configure

COPY ./experiment_bridge_entrypoint.sh /