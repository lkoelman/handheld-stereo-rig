# How to setup

To be able to replay the pcap files we need to do the following steps in order:

1. Build the docker container

```bash
$ cd ../ros2_ouster
$ docker build -t ouster_pub .
```

2. Download the data from the [gdrive link](https://drive.google.com/drive/folders/1NJnHycEOhWNSuB6Oxrwr6BZ7clOp7dDD?usp=sharing)

> **_INFO:_**  Every Reifly members should have access to the shared folder

3. Start the container

```bash
$ docker run -it --name ouster_pub /bin/bash
```

4. Create a specific network config for the docker container and attach it

> **_INFO:_**  The aim of this step to configure the mtu buffers with the enough size, so that the packages can be replayed without any drop

```bash
$ docker network create --opt com.docker.network.driver.mtu=13000 mtu-extended-network
$ docker network connect mtu-extended-network ouster_pub
```

5. Copy the data from the host pc to the container

```bash
$ docker cp . ouster_pub:/home/ubuntu
```

# How to run

1. Source the development environment

```bash
$ source /home/ubuntu/dev_ws/install/setup.bash
```

2. Open two more terminals and attach to the same running container via following command

> **_INFO:_**  Do the following for each terminal tab

```bash
$ docker exec -it ouster_pub /bin/bash
$ source /home/ubuntu/dev_ws/install/setup.bash
```

3. Find the network interface that is created with `13000` buffer size

```bash
ifconfig | grep mtu
```

4. Once the NIC has been found, update the `tins_driver.yaml` config with the following lines:

```bash
ros__parameters:
    lidar_ip: laser_ip
    computer_ip: computer_ip   
    lidar_mode: "512x10"
    imu_port: 7503
    lidar_port: 7502
    sensor_frame: laser_sensor_frame
    laser_frame: laser_data_frame
    imu_frame: imu_data_frame
    
    # The ethernet device the packets are being broadcast through. Only used
    # by the "tins" driver type.
    ethernet_device: nic_name
```

## To run the ouster_driver

We already sourced the workspace so don't need to repeat that for any of the 

 Run the ros driver to start to consume some data

```bash
ros2 launch ros2_ouster tins_driver_launch.py metadata_filepath:=/home/dev_ws/src/ouster_pub/data/ouster_data/OS-1-64-U13_122108000334_512x10_20220727_181114.json params_file:=/home/dev_ws/src/ouster_pub/ros2_ouster/params/tins_driver_config.yaml
```

## To replay the captured data

```bash
$ tcpreplay -v --intf1=eth2 /home/ubuntu/tcpdump_long_fw-v2.3.0.pcap
```

## To record or check the data

### To record

```bash
$ ros2 bag record -a
```

### To check

```bash
$ ros2 topic bw /scan
```
