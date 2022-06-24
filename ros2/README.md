A Docker environment for running the depthai ROS [bindings](https://github.com/luxonis/depthai-ros#getting-started) and [examples](https://github.com/luxonis/depthai-ros-examples)


## Usage

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