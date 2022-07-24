# handheld-stereo-rig

Handheld stereo + lidar + IMU data collection

# Setup

## OAK-D

Install depthai dependencies:

```sh
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash
```

Installation from Pypi:
```sh
python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai
```

Installation from source:

```sh
git clone git@github.com:luxonis/depthai.git
python3 -m venv .venv
source .venv/bin/activate
python3 depthai/install_requirements.py

# Either modify pythonpath
export PYTHONPATH=${PYTHONPATH}:$(pwd)/depthai
export PYTHONPATH=${PYTHONPATH}:$(pwd)/depthai/depthai_sdk/src

# Or add following to start of your python code
sys.path.append(path/to/depthai)
sys.path.append(str((Path("path/to/depthai") / "depthai_sdk" / "src").absolute()))
```

### ROS2

See https://github.com/luxonis/depthai-ros#getting-started

The Dockerfile contains the ROS2 node and examples from https://github.com/luxonis/depthai-ros-examples

Usage:

```sh
cd docker-ros-official
docker_build.sh
docker_run.sh

# Run example
cd $HOME/dai_ws
colcon build
source install/setup.bash
ros2 launch depthai_examples rgb_stereo_node.launch.py
```

## VN100

Serial communication with the VN100 is implemented in a cpp app encapsulated
by a ROS2 node.

The instructions below show how to build a Docker image containing
a ROS2 environment with the VN100 node.


```sh
# Ensure that the serial device ID is set correctly in the .cpp file
# and exposed to the docker container
ls /dev/serial/by-id/usb-FTDI*
# Note down the device ID (example)
DEVICE="/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FT1WD85D-if00-port0"

# Build and run the docker image
cd ros2/docker-vn100
docker build -t vn100_pub .
docker run -it --device=${DEVICE} vn100_pub bash
```

**Inside the docker container**

```sh
source /home/ubuntu/dev_ws/install/setup.bash
ros2 run vn100_pub vn100pub
```
