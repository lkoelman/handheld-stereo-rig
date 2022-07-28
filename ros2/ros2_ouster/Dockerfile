
FROM ros:galactic

ARG USERNAME=ubuntu
ARG WORKSPACE=/home/ubuntu/dev_ws
ARG DEBIAN_FRONTEND=noninteractive

COPY ouster_pub ${WORKSPACE}/src
RUN chmod +666 ${WORKSPACE}
#RUN chown -R ${USERNAME}:${USERNAME} ${WORKSPACE}

RUN bash -c "apt-get update"
RUN bash -c "apt-get -y install build-essential wget nano tcpreplay"

# Install some dependencies
RUN bash -c "rosdep update"
RUN cd ${WORKSPACE} \
    && rosdep install -r -y -i --from-paths src \
    && colcon build

RUN cd ${WORKSPACE} \
    && . /opt/ros/galactic/setup.sh \
    && colcon build 
    
RUN bash -c "source ${WORKSPACE}/install/setup.bash"

# the startup script needs to run as root
USER root