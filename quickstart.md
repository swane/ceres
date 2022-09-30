# Quick start 
## Hardware
- Boot Ceres, ensure the router is powered and connected to Pi
- Connect the crocodile clips to the 12V battery (powers the Arduino and motors). Connect the loops to the battery (powers the Pi)
- Ensure USB for Arduino and GPS are both connected to Pi
## Dell PC
- Connect  Dell PC to wifi using SSID NETGEAR (no pwd)
- Run VNC on Dell, connect to Pi (192.168.0.2)
- On Pi, use ifconfig to check IP address is (192.168.0.2)
- On Dell use gedit .bashrc ensure ROS Master (192.168.0.2) and ROS IP  (192.168.0.10 own Dell IP)
- Restart terminal if .bashrc altered
- On the Dell, Navigate to catkin_ws/src/ceres/src open terminal here
## Pi on the robot
- roslaunch ublox_gps ardusimple.launch
- check back on Dell ‘rostopic list’
- if there is an error, unplug all USB on Pi, plug in GPS first, then Arduino
- On Pi, Navigate to catkin_ws/src/ceres/src and run 2ceres_ros.py (./2Ceres_Ros)
- On Pi run ./test_steer_pub.py to test steering
- Stop the test_steer_pub.py
- On Pi:
- Run ./ 4_nav_path_points.py
- On Dell run pub_4_path_test.py
## Publishing the paths
![Paths](pubpaths.png)
