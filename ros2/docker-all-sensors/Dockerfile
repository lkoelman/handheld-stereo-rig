ARG ROS_DISTRO=galactic
FROM ros:$ROS_DISTRO-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    wget \
    git \
    nano \
    apt-utils \
    build-essential

# For OAK-D: compile libusb without udev support
# Same as in Docker base images python:3.9-bullseye, ubuntu:20.04, nvidia/cuda:*-ubuntu20.04
# See https://discuss.luxonis.com/d/533-oak-d-lite-not-working-with-official-docker-container/12
# RUN apt-get remove -y libusb-1.0-0
RUN wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2 && \
    tar xf libusb-1.0.24.tar.bz2 && \
    cd libusb-1.0.24 && \
    ./configure --disable-udev && \
    make -j && make install

# ROS2 dependencies for OAK-D and Ouster
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-depth-image-proc \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    libyaml-cpp-dev
    # os-$ROS_DISTRO-mavros* \
    # ros-$ROS_DISTRO-vision-opencv \

# APT dependencies for OAK-D
ADD docker-all-sensors/install_depthai_deps.sh /
RUN chmod +x /install_depthai_deps.sh
RUN /install_depthai_deps.sh

# Install DepthAI ROS dependencies
# See https://github.com/luxonis/depthai-ros#getting-started
# This also installs https://github.com/luxonis/depthai-ros-examples
# NOTE: tested with main = commit b7d9dd82bdd08af3fe929347a493c1ebf527f350
RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | bash
RUN mkdir -p $HOME/dev_ws/src && cd $HOME/dev_ws/src

# Clone depthai
RUN git clone https://github.com/luxonis/depthai-ros.git && \
    cd $HOME/dev_ws && \
    rosdep install --from-paths src --ignore-src -r -y

#RUN git clone https://github.com/luxonis/depthai-ros.git && \
#    cd depthai-ros && \
#    git checkout ros-release

#RUN cd $HOME/dev_ws/src && \
#    git clone https://github.com/luxonis/depthai-ros-examples.git && \
#    cd depthai-ros-examples && \
#    git checkout main

#RUN cd $HOME/dev_ws && \
#    rosdep install --from-paths src --ignore-src -r -y

#RUN apt install ros-galactic-depthai-ros
#RUN wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos && \
#    vcs import src < underlay.repos && \
#    rosdep install --from-paths src --ignore-src -r -y

# Install Ouster OS-1 ROS2 drivers
# NOTE: tested with main = commit d3cd586e2548f9989b68ebc33b03a5d99b816ea8
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    libpcap-dev libssl-dev libtins-dev && \
    cd $HOME/dev_ws/src && \
    git clone https://github.com/ros-drivers/ros2_ouster_drivers.git

# Install VN100 IMU driver
RUN apt-get install -y -qq --no-install-recommends \
    libboost-all-dev
WORKDIR /root/dev_ws/src
COPY ./vn100_pub ./vn100_pub/

# Install Data Recorder
COPY ./record_all_data ./record_all_data

# Install launch files
COPY ./launch/ ./launch/

# Build ROS nodes
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd $HOME/dev_ws && \
    colcon build

# RUN rm -rf /var/lib/apt/lists/* && \
#     apt-get clean
RUN mkdir /root/data_log

# Setup entrypoint
ENV ROS_DISTRO=$ROS_DISTRO
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc
COPY docker-all-sensors/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
#ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]
# We would like to run the entrypoint instead of initialising it
CMD ["/ros_entrypoint.sh"]