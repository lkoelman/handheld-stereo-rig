#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ -z "$@" ]]
then
    echo "No launch parameters specified. Starting all sensors ..."
    source $HOME/dev_ws/install/setup.bash
    # Launch file must be fully qualitifed path or <package> <launch file>
    ros2 launch $HOME/dev_ws/src/launch/all-sensors.launch.yaml
else
    exec "$@"
fi
