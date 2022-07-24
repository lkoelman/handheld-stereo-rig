#!/bin/bash

DEVICE="/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FT1WD85D-if00-port0"

# Run the entrypoint without CMD argument
docker run --rm \
    -it \
    --device=${DEVICE} \
    --net=host \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    reifly-ros2-nodes:galactic-ros-base
