ARG ROS_DISTRO=galactic
FROM ros:$ROS_DISTRO-ros-base

RUN apt-get update && \
    apt-get install -y -qq \
    wget \
    apt-utils

# Use libusb without udev support
# Same as in Docker base images python:3.9-bullseye, ubuntu:20.04, nvidia/cuda:*-ubuntu20.04
# See https://discuss.luxonis.com/d/533-oak-d-lite-not-working-with-official-docker-container/12
# RUN apt-get remove -y libusb-1.0-0
RUN wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2 && \
    tar xf libusb-1.0.24.tar.bz2 && \
    cd libusb-1.0.24 && \
    ./configure --disable-udev && \
    make -j && make install


RUN apt-get update && \
    apt-get install -y -qq \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-mavros* \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-depth-image-proc \
    libyaml-cpp-dev && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc

# Install apt dependencies
ADD install_depthai_deps.sh /
RUN chmod +x /install_depthai_deps.sh
RUN /install_depthai_deps.sh

# Install ROS dependencies from https://github.com/luxonis/depthai-ros#getting-started
# This also installs https://github.com/luxonis/depthai-ros-examples
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash && \
    mkdir -p $HOME/dai_ws/src && cd $HOME/dai_ws && \
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
