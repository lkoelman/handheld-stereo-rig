#!/bin/bash

docker run --rm \
    -it \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    reifly-oakd-node:latest \
    bash
