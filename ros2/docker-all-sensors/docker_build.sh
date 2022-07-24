#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
ROS2_ROOT=$(realpath "${SCRIPT_DIR}/..")

# Build ROS container from parent context
cd $ROS2_ROOT
echo "Building Dockerfile from root dir $(pwd)"
docker build -t reifly-ros2-nodes:galactic-ros-base -f docker-all-sensors/Dockerfile .
