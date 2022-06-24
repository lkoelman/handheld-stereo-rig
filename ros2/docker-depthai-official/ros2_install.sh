#!/usr/bin/env bash
set -eu

# REF: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
# by Open Robotics, licensed under CC-BY-4.0
# source: https://github.com/ros2/ros2_documentation

ROS_DISTRO=galactic
INSTALL_PACKAGE=ros-base
TARGET_OS=focal

# Check OS version
if ! which lsb_release > /dev/null ; then
	apt-get update
	apt-get install -y curl lsb-release
fi

DISTRO=$(lsb_release -sc)
if [[ "${DISTRO}" == "$TARGET_OS" ]]; then
	echo "OS Check Passed"
else
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This OS (version: $(lsb_release -sc)) is not supported"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

# Install
apt-get update
apt-get install -y -qq software-properties-common
add-apt-repository universe
apt-get install -y -qq curl gnupg2 lsb-release build-essential

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt-get update
apt-get install -y -qq ros-$ROS_DISTRO-$INSTALL_PACKAGE
apt-get install -y -qq python3-argcomplete \
	python3-colcon-common-extensions \
	python3-rospkg python3-rosdistro \
	python3-catkin-pkg python3-rosdep \
	python3-vcstool # https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/

[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] ||
rosdep init
rosdep update
grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc ||
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Extra ROS packages
apt-get install -y -qq \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-mavros* \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-depth-image-proc \
    libyaml-cpp-dev

set +u

# source /opt/ros/$ROS_DISTRO/setup.bash
# echo "success installing ROS2 $ROS_DISTRO"
# echo "Run 'source /opt/ros/$ROS_DISTRO/setup.bash'"