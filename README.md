# handheld-stereo-rig
Handheld stereo + lidar + IMU data collection


# Setup 

## OAK-D 

```sh
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash
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

## VN100

After many attempts with Serial configuration we decided to move forward with a cpp app

to set the comms with the device. Commands given below will help the user to set up a ros2

environment that builds the app. 

```sh
cd cpp/
docker build -t vn100_pub .
docker run --it vn100_pub /bin/bash 
```

**Inside the docker container**
```sh
source /home/ubuntu/dev_ws/install/setup.bash
ros2 run vn100_pub vn100pub
```