#!/bin/bash

DATA_DIR="${HOME}/Documents/OAK-D"
mkdir -p ${DATA_DIR}

xhost local:root

docker run --rm \
    -it \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${DATA_DIR}:/data \
    ros2-depthai:galactic-depthai-lib \
    bash