ARG ROS_DISTRO=galactic
FROM ros:$ROS_DISTRO-ros-base

RUN apt-get update && \
    apt-get install -y -qq \
    wget \
    git \
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
    apt-get install -y -qq --no-install-recommends \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-mavros* \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-depth-image-proc \
    ros-$ROS_DISTRO-pcl-conversions
    libyaml-cpp-dev && \
    echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc

# Install apt dependencies
ADD install_depthai_deps.sh /
RUN chmod +x /install_depthai_deps.sh
RUN /install_depthai_deps.sh

# Install DepthAI ROS dependencies from https://github.com/luxonis/depthai-ros#getting-started
# This also installs https://github.com/luxonis/depthai-ros-examples
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash

# Install OAK-D ROS2 packages
RUN mkdir -p $HOME/dev_ws/src && cd $HOME/dev_ws && \
    wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos && \
    vcs import src < underlay.repos && \
    rosdep install --from-paths src --ignore-src -r -y

# Install Ouster OS-1 ROS2 drivers
RUN apt-get install -y -qq --no-install-recommends \
    libpcap-dev libssl-dev libtins-dev && \
    cd $HOME/dev_ws && \
    git clone https://github.com/ros-drivers/ros2_ouster_drivers.git

# Install VN100 IMU driver
RUN apt-get install -y -qq --no-install-recommends \
    libboost-all-dev
COPY ../vn100_pub $HOME/dev_ws/src

# Build ROS nodes
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
#     cd $HOME/dev_ws && \
#     colcon build && \
#     source install/setup.bash

# RUN rm -rf /var/lib/apt/lists/* && \
#     apt-get clean
