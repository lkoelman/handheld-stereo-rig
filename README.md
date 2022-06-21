# handheld-stereo-rig
Handheld stereo + lidar + IMU data collection


# Setup 

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
