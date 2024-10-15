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
wget pip \
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
ros-noetic-usb-cam \
ros-noetic-roscpp-tutorials=0.10.2-1* \
ros-noetic-rospy-tutorials=0.10.2-1* \
&& rm -rf /var/lib/apt/lists/* 

# Clone and build custom ROS1 messages from source 
RUN mkdir -p /ros1_ws/src 
WORKDIR /ros1_ws/src 
RUN git clone -b noetic-devel https://github.com/ros-perception/ar_track_alvar.git
WORKDIR /ros1_ws 
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash && \ 
catkin_make

COPY ./ros1_ar_entrypoint.sh /
COPY ./ros1_cam_entrypoint.sh /