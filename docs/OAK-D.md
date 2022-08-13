# Installation

## Sensor installation

References:
- https://docs.luxonis.com/projects/api/en/latest/install/#ubuntu
- https://docs.luxonis.com/en/latest/pages/troubleshooting/

In Linux OS, run these commands to give USB permissions for the regular user:

```sh
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify installation after plugging in OAK-D:

```
$ lsusb | grep 03e7
Bus 001 Device 120: ID 03e7:2485 Intel Movidius MyriadX
```

```
$ dmesg -H
...
[ +19.059354] usb 8-1: new high-speed USB device number 5 using xhci_hcd
[  +0.152351] usb 8-1: New USB device found, idVendor=03e7, idProduct=2485
[  +0.000006] usb 8-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  +0.000003] usb 8-1: Product: Movidius MyriadX
[  +0.000003] usb 8-1: Manufacturer: Movidius Ltd.
[  +0.000002] usb 8-1: SerialNumber: 03e72485
```

## Python development

Install dependencies:

```sh
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash
```

Installation from Pypi:
```sh
python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai
```

There are two official repos
- [DepthAI SDK](https://github.com/luxonis/depthai)
    - contains the Python GUI code
- [DepthAI API](https://github.com/luxonis/depthai-python)
    - contains all the python examples

## ROS2 C++ nodes

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

# Testing

Docker container testing

```sh
ln -s /usr/local/bin/pip3.8 /usr/bin/pip3.8
pip install ipython
python3 /data/depthai-python/examples/install_requirements.py
```

Local testing

```sh
git clone https://github.com/luxonis/depthai.git
git clone https://github.com/luxonis/depthai-python.git

conda create -n oakd python=3.9 pip numpy scipy Pillow ipython
conda activate oakd

python3 depthai-python/examples/install_requirements.py
python3 depthai/install_requirements.py
python3 -m pip install -r depthai/depthai_sdk/requirements.txt
```

Run GUI: `python3 depthai/depthai_demo.py`
Run stereo example: `python3 depthai-python/examples/StereoDepth/stereo_depth_video.py`

# Configuration

https://docs.luxonis.com/projects/api/en/latest/tutorials/image_quality/