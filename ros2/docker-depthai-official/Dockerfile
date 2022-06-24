# Can use python:3.9-bullseye, ubuntu:20.04, nvidia/cuda:*-ubuntu20.04
# FROM ubuntu:focal
FROM nvidia/cuda:11.7.0-devel-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

# --- Contents of official depthai Dockerfile
# See https://github.com/luxonis/depthai-python/blob/develop/ci/Dockerfile
RUN apt-get update && apt-get install -y -qq apt-utils wget build-essential cmake pkg-config libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran git python3-pip
ADD docker_dependencies.sh .
RUN chmod +x ./docker_dependencies.sh && ./docker_dependencies.sh
RUN pip install -U pip && \
    pip install --extra-index-url https://www.piwheels.org/simple/ --prefer-binary opencv-python && \
    pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai
# --- End official depthai Dockerfile contents

# Install ROS2
ADD ros2_install.sh /
RUN chmod +x /ros2_install.sh && \
    /ros2_install.sh

# Install ROS dependencies from https://github.com/luxonis/depthai-ros#getting-started
# This also installs https://github.com/luxonis/depthai-ros-examples
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash && \
    mkdir -p $HOME/dai_ws/src && cd $_ && \
    wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos && \
    vcs import src < underlay.repos && \
    rosdep install --from-paths src --ignore-src -r -y
    
# Build ROS nodes
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
#     cd $HOME/dai_ws && \
#     colcon build && \
#     source install/setup.bash

# RUN rm -rf /var/lib/apt/lists/* && \
#     apt-get clean