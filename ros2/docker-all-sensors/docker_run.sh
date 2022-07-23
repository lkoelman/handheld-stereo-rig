#!/bin/bash

docker run --rm \
    -it \
    --privileged \
    --net=host \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros2-reifly:galactic-ros-base \
    bash
