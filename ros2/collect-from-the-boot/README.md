# Data Collection from the boot

This file describes the intended approach to be able to achieve Data Collection from the boot. 

The current approach consists of two seperate parts:

1. The docker container kicks off data collection once it starts to run

That has been achieved by introducing following lines in the Dockerfile:

```bash
COPY docker-all-sensors/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
#ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]
# We would like to run the entrypoint instead of initialising it
CMD ["/ros_entrypoint.sh"]
```

2. The docker container runs as a systemd service and the service starts the docker container once the compute has been booted.

`docker service file`

```bash
[Unit]
Description=HandPod Data Collection Service
After=docker.service
Requires=docker.service

[Service]
TimeoutStartSec=0
Restart=always
ExecStartPre=/usr/bin/docker stop reifly-all-node-data-collector 
ExecStartPre=/usr/bin/docker rm reifly-all-node-data-collector
ExecStart=/usr/bin/docker run --rm --name  reifly-all-node-data-collector \
    --device="/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FT1WD85D-if00-port0" \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /home/pilot/data_log:/root/data_log \
    --device-cgroup-rule='c 189:* rmw' \   
    reifly-ros2-nodes:galactic-ros-base
ExecStopPost= /usr/bin/docker stop reifly-all-node-data-collector
ExecStopPost=/usr/bin/docker rm reifly-all-node-data-collector
[Install]
WantedBy=default.target
```

## To start the service

1. Create file as `sudo touch /etc/systemd/system/docker.handpod.service`

2. Copy the above content to the service file

3. Enable service file via

`systemctl enable docker.handpod`

4. Start the service

`sudo service docker.handpod start`

5. If you want to check the status

`sudo service docker.handpod status`

6. If you want to stop the service

`sudo service docker.handpod stop`