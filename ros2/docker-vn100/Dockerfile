# NOTE: build this from parent folder of vn100_pub using `docker build -f /path/to/Dockerfile`
FROM ros:galactic

ARG USERNAME=ubuntu
ARG WORKSPACE=/home/ubuntu/dev_ws


COPY vn100_pub/ ${WORKSPACE}/src/vn100_pub/
RUN chmod +666 ${WORKSPACE}
#RUN chown -R ${USERNAME}:${USERNAME} ${WORKSPACE}

RUN bash -c "apt-get update"
RUN bash -c "apt-get -y install build-essential wget nano"

# Install some dependencies
RUN bash -c "apt-get -y install libboost-all-dev"

RUN cd ${WORKSPACE} \
    && . /opt/ros/galactic/setup.sh \
    && colcon build

RUN bash -c "source ${WORKSPACE}/install/setup.bash"

# the startup script needs to run as root
USER root


