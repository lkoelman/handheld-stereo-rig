#!/bin/bash

# Build ROS container
docker build -t ros2-reifly:galactic-ros-base -f Dockerfile .
