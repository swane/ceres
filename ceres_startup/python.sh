#! /bin/bash

source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
source /home/pi/fp_public_ws/devel/setup.bash

python /home/pi/catkin_ws/src/ceres/src/2Ceres_ROS.py


read -p "Oops.. was the USB plugged in?"