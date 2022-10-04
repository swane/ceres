# publish points as Paths <br>
```
#!/usr/bin/env python
#https://roboticsbackend.com/ros-topic-command-line-tools-practical-example-rostopic-and-rosmsg/


import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('publish_path')
pub=rospy.Publisher('positions',Path,queue_size=10,latch=True)

rate=rospy.Rate(2)
msg = Path()
msg.header.frame_id = "map"
msg.header.stamp = rospy.Time.now()

lat=52.782178
lon=-2.429445
pose = PoseStamped()
msg.header.stamp = rospy.Time.now()
pose.pose.position.x = lat
pose.pose.position.y = lon
pose.pose.position.z = 0

pose.pose.orientation.x = 0
pose.pose.orientation.y = 0
pose.pose.orientation.z = 0
pose.pose.orientation.w = 1
msg.poses.append(pose)
print lat
print lon
lat=52.782438
lon=-2.429827
pose = PoseStamped()
msg.header.stamp = rospy.Time.now()
pose.pose.position.x = lat
pose.pose.position.y = lon
pose.pose.position.z = 0

pose.pose.orientation.x = 0
pose.pose.orientation.y = 0
pose.pose.orientation.z = 0
pose.pose.orientation.w = 1
msg.poses.append(pose)
print lat
print lon

lat=52.782357
lon=-2.429999
pose = PoseStamped()
msg.header.stamp = rospy.Time.now()
pose.pose.position.x = lat
pose.pose.position.y = lon
pose.pose.position.z = 0

pose.pose.orientation.x = 0
pose.pose.orientation.y = 0
pose.pose.orientation.z = 0
pose.pose.orientation.w = 1
msg.poses.append(pose)
print lat
print lon
pose = PoseStamped()
msg.header.stamp = rospy.Time.now()
lat=52.782273
lon=-2.429882

pose.pose.position.x = lat
pose.pose.position.y = lon
pose.pose.position.z = 0

pose.pose.orientation.x = 0
pose.pose.orientation.y = 0
pose.pose.orientation.z = 0
pose.pose.orientation.w = 1
msg.poses.append(pose)
print lat
print lon
pose = PoseStamped()
msg.header.stamp = rospy.Time.now()
lat=52.782178
lon=-2.429445

pose.pose.position.x = lat
pose.pose.position.y = lon
pose.pose.position.z = 0

pose.pose.orientation.x = 0
pose.pose.orientation.y = 0
pose.pose.orientation.z = 0
pose.pose.orientation.w = 1
msg.poses.append(pose)
rate.sleep()
print lat
print lon
print "Publishing now"
pub.publish(msg)
rate.sleep()
rate.sleep()
rate.sleep()
rate.sleep()
rate.sleep()
```
Path read by nav program<br>
# Read kml and publish points
```
#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 10:44:03 2022
@author: 00751570
Convert Google MyMaps kml file into points or arrays of points for Sam's robots to follow
Updated to include lineStrings 12/09/2022
"""

import xml.etree.ElementTree as ET
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('publish_path')
pub=rospy.Publisher('positions',Path,queue_size=10,latch=True)

rate=rospy.Rate(2)
msg = Path()
msg.header.frame_id = "map"
msg.header.stamp = rospy.Time.now()

tree = ET.parse('path_to_4G.kml') # change to suit file movement
root = tree.getroot()


for placecemark in root.iter('{http://www.opengis.net/kml/2.2}Placemark'):
    placemark_name = placecemark[0].text

    print()
    print(placemark_name)
    movement = placecemark[2].tag
    #print(movement)
    
    if 'Point' in movement:
        print('Point: ')
        #print(placecemark[2].tag)
        #print(placecemark[2][0].text)
        point = (placecemark[2][0].text).strip().replace(' ', '').replace(',0', '')
        point_as_list = list(filter(bool, point.splitlines()))
        
        print(point_as_list)
        #print()

    if 'LineString' in movement:
        print('Line string: ')
        #print(placecemark[2].tag)
        coords_array = (placecemark[2][1].text).strip().replace(' ', '').replace(',0', '')
        coord_list = list(filter(bool, coords_array.splitlines()))
        
        coarr = []

        for item in coord_list:
            item = item.split(',')
            coarr.append(item)
            print(item[1]) #lat
            print(item[0]) #lon
            pose = PoseStamped()
            pose.pose.position.x =float(item[1])
            pose.pose.position.y = float(item[0])
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            msg.poses.append(pose)
	    rate.sleep()
        print "Publishing now"
        pub.publish(msg)
        
        
    if 'Polygon' in movement:
        print('Polygon: ')
        #print(placecemark[2].tag)
        coords_array = (placecemark[2][0][0][1].text).strip().replace(' ', '').replace(',0', '')
        coord_list = list(filter(bool, coords_array.splitlines()))
        
        coarr = []

        for item in coord_list:
            item = item.split(',')
            coarr.append(item)
            #print(item)
        
        print(coarr)
```
# PID
http://wiki.ros.org/pid
```
$ roslaunch pid servo_sim.launch
```
## Dynamic reconfigure
```
rosrun rqt_reconfigure rqt_reconfigure
```
```
<node name="controller" pkg="pid" type="controller" ns="left_wheel" output="screen" >
      <param name="node_name" value="left_wheel_pid" />
      <param name="Kp" value="5.0" />
      <param name="Ki" value="0.0" />
      <param name="Kd" value="0.1" />
      <param name="upper_limit" value="10" />
      <param name="lower_limit" value="-10" />
      <param name="windup_limit" value="10" />
      <param name="max_loop_frequency" value="100.0" />
      <param name="min_loop_frequency" value="100.0" />
      <remap from="setpoint" to="/setpoint" />
     </node>
```
## Topics
- setpoint: The controller subscribes to this topic and reads std_msgs/Float64 messages. The message data element must contain the desired value of the state measurement of the controlled process.
- state: The controller subscribes to this topic and reads std_msgs/Float64 messages. The message data element must contain the current value of the controlled plant property. The controller publishes std_msgs/Float64 messages on the control_effort topic each time it receives a message on the state topic. Thus the rate at which the plant publishes state governs the control-loop rate - the plant should publish state at the desired loop rate.
- control_effort: The control_effort message data element contains the control effort to be applied to the process to drive state/data to equal setpoint/data. 

# Parse_send_path.py
Reads a .kml (Google maps) and sends it as a Path topic

# Send pose data
```
#!/usr/bin/env python

import rospy


from geometry_msgs.msg import PoseStamped



def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('test_poses',anonymous=False)
    
 
pubPose=rospy.Publisher("/UTMposes", PoseStamped, queue_size=1)

x=0
y=0
while not rospy.is_shutdown():
        x=x+5
        y=y+7
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "UTM"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pubPose.publish(pose)
	rospy.sleep(1)

```
## Subscribe to poses and write as csv file
```
#!/usr/bin/env python

import rospy
import csv

from geometry_msgs.msg import PoseStamped

#f = open("demofile.txt", "w")
ef = open('points_visited.csv', mode='w') 
ew = csv.writer(ef, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
def posecallback(msg):
	a = msg.pose.position.x
	b = msg.pose.position.y
	ew.writerow([str(a), str(b)])

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	ef.close()
	self.cmd_vel.publish(Twist())

rospy.init_node('save_poses',anonymous=False)   
 
pubPose=rospy.Subscriber("/UTMposes", PoseStamped, posecallback)

while not rospy.is_shutdown():   
	rospy.sleep(0.1)
```

