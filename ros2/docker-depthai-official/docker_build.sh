#!/bin/bash

# Build ROS container
docker pull luxonis/depthai-library
docker build -t ros2-depthai:galactic-depthai-lib -f Dockerfile .