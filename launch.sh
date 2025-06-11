#!/bin/bash

chmod 774 /dev/ttyUSB0
source /home/robot/桌面/2025/lh/lidar/mid360_ws/devel/setup.bash
roslaunch serial_sender serial_sender.launch 
