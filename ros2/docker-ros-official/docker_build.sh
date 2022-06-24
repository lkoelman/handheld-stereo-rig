#!/bin/bash

# Build ROS container
docker build -t ros2-depthai:galactic-ros-base -f Dockerfile .