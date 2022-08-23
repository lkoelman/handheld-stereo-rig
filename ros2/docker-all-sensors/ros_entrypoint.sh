#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# To get rid of failing DDS issue
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [[ -z "$@" ]]
then
    # Get the current time/date
    echo "Getting the current time/date ..."
    CURR_TIME=$(date +%Y%m%d_%H%M%S)
    echo $CURR_TIME

    LOG_DIR=/root/data_log/${CURR_TIME}_$RANDOM
    # Create folder with time/datae
    mkdir -p $LOG_DIR
    export ROS_LOG_DIR=$LOG_DIR

    # Specify ROS LOG folder with the same time/date
    cd $LOG_DIR
    echo "Folder has been created. The new directory:"
    echo $(pwd)

    echo "No launch parameters specified. Starting all sensors ..."
    source $HOME/dev_ws/install/setup.bash
    # Launch file must be fully qualitifed path or <package> <launch file>
    ros2 launch $HOME/dev_ws/src/launch/all-sensors.launch.yaml
else
    exec "$@"
fi