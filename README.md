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

### Unit Setup 

**(!!Attention - Do it before attempting to use  the driver)**

You can find step-by-step how to configure the VN100 IMU for our given use-case. The given configs will set the unit to publish:

- TimeStartup (0th bit in Common Group)

- Yaw/Pitch/Roll (3rd bit in Common Group)

- Quaternion (4th bit in Common Group)

- Compensated Angular Rate (5th bit in Common Group)

The configs have been slightly changed from [python-vectornav-repo](https://github.com/fdcl-gwu/python-vectornav). 

#### **Copied from Repo** 

You can use any serial communication software (e.g.: Serial Monitor on Arduino IDE) to set following configurations on the IMU. **If you are using the Arduino Serial Monitor, make sure to change the line ending setting in the bottom of the Serial Monitor to Newline.** If you need any other parameters, you must change the group field values as described in the sensor manual, as well as the message parsing on VectorNav class.

```bash
$VNASY,0*XX                 // stop async message printing
$VNWRG,06,0*XX              // stop ASCII message outputs
$VNWRG,75,2,16,01,039*XX    // output binary message (see below for details)
$VNCMD*XX                   // enter command mode
system save                 // save settings to flash memory
exit                        // exit command mode
$VNASY,1*XX                 // resume async message printing
```

#### **Configuring IMU Binary Message**
Command | Register | Serial Port | Frequency | Group | Output Message | Checksum
------- | -------- | ----------- | --------- | ----- | -------------- | ---------
$VNWRG  | 75       | 2           | 16         | 01    | 039           | *XX
Write to register command | Register number for the output binary message | This depends on the cable you use | divide 800 by this value get the required frequency, this example is for 50 Hz | Output group as defined in the manual | data categories from the group, this example is for b000011101 = 0x039 | Use XX for unknown checksum values 

#### **Running the Code**
1. Configure the IMU as described above.
2. Update the serial port and baud rates in test_vn100.py
3. Run the code: `python test_vn100.py`

### Driver Setup

Install the **serial** module