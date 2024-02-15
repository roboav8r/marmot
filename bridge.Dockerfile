# Install ROS2 from source from these instructions:
# https://docs.ros.org/en/galactic/Installation/Alternatives/Ubuntu-Development-Setup.html

# This is a modified version of the original dockerfile here, made to work with Ubuntu 20/galactic:
# https://github.com/osrf/docker_images/blob/master/ros2/source/devel/Dockerfile


# #################################################################################
# # Install ROS2 from source
# FROM ubuntu:focal-20231211

# SHELL ["/bin/bash", "-c"]

# # setup timezone
# RUN echo 'Etc/UTC' > /etc/timezone && \
#     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     apt-get update && \
#     apt-get install -q -y --no-install-recommends tzdata && \
#     rm -rf /var/lib/apt/lists/*

# # install packages
# RUN apt-get update && apt-get install -q -y --no-install-recommends \
#     bash-completion \
#     dirmngr \
#     gnupg2 \
#     python3-flake8 \
#     # python3-flake8-blind-except \
#     # python3-flake8-builtins \
#     # python3-flake8-class-newline \
#     # python3-flake8-comprehensions \
#     # python3-flake8-deprecated \
#     # python3-flake8-docstrings \
#     # python3-flake8-import-order \
#     # python3-flake8-quotes \
#     python3-pip \
#     # python3-pytest-repeat \
#     # python3-pytest-rerunfailures \
#     && rm -rf /var/lib/apt/lists/*

# # setup keys
# RUN set -eux; \
#        key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
#        export GNUPGHOME="$(mktemp -d)"; \
#        gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
#        mkdir -p /usr/share/keyrings; \
#        gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
#        gpgconf --kill all; \
#        rm -rf "$GNUPGHOME"

# # setup sources.list
# RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# # setup environment
# ENV LANG C.UTF-8
# ENV LC_ALL C.UTF-8

# # install bootstrap tools
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     build-essential \
#     git \
#     python3-colcon-common-extensions \
#     python3-colcon-mixin \
#     python3-rosdep \
#     python3-setuptools \
#     python3-vcstool \
#     && rm -rf /var/lib/apt/lists/*

# # install python packages
# RUN pip3 install -U \
#     argcomplete
# # This is a workaround for pytest not found causing builds to fail
# # Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
# RUN pip3 freeze | grep pytest \
#     && python3 -m pytest --version

# # bootstrap rosdep
# RUN rosdep init \
#     && rosdep update

# # setup colcon mixin and metadata
# RUN colcon mixin add default \
#       https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
#     colcon mixin update && \
#     colcon metadata add default \
#       https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
#     colcon metadata update

# # clone source
# ENV ROS2_WS /opt/ros2_ws
# RUN mkdir -p $ROS2_WS/src
# WORKDIR $ROS2_WS

# # build source
# RUN colcon \
#     build \
#     --cmake-args \
#       -DSECURITY=ON --no-warn-unused-cli \
#     --symlink-install

# # setup bashrc
# RUN cp /etc/skel/.bashrc ~/

# # setup entrypoint
# COPY ./ros2_source_entrypoint.sh /

# # Source ROS2 workspace
# RUN source ${ROS2_WS}/install/setup.bash

# # ENTRYPOINT ["/ros2_source_entrypoint.sh"]
# # CMD ["bash"]

# # set environment
# ARG ROS2_DISTRO=galactic
# ENV ROS_DISTRO=$ROS2_DISTRO
# ENV ROS2_VERSION=2 \
#     ROS2_PYTHON_VERSION=3

# # clone source
# ARG ROS2_BRANCH=$ROS2_DISTRO
# ARG ROS2_REPO=https://github.com/ros2/ros2.git
# RUN git clone $ROS2_REPO -b $ROS2_BRANCH \
#     && vcs import src < ros2/ros2.repos

# # install dependencies
# RUN apt-get update && rosdep install -y \
#     --from-paths src \
#     --ignore-src \
#     --skip-keys " \
#         fastcdr \
#         rti-connext-dds-6.0.1 \
#         urdfdom_headers" \
#     && rm -rf /var/lib/apt/lists/*

# # build source
# RUN colcon \
#     build \
#     --symlink-install \
#     --mixin build-testing-on release \
#     --cmake-args --no-warn-unused-cli

# # test build
# ARG RUN_TESTS
# ARG FAIL_ON_TEST_FAILURE
# RUN if [ ! -z "$RUN_TESTS" ]; then \
#         colcon test; \
#         if [ ! -z "$FAIL_ON_TEST_FAILURE" ]; then \
#             colcon test-result; \
#         else \
#             colcon test-result || true; \
#         fi \
#     fi





####################################################################################3
# JUST THE BRIDGE APPROACH

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
