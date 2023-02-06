#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from math import pi, sin, cos, tan, sqrt, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from latlongtoutm import *
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from ublox_msgs.msg import NavSTATUS, NavRELPOSNED9
import utm
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped

x_speed = 1.0  # 0.1 m/s
z_ang=0.577  #20 deg / sec
# this quick check means that the following code runs ONLY if this is the 
# main file -- if we "import move" in another file, this code will not execute.


    



def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('test_pub_steertwist',anonymous=False)
   
# publish to /yocs_cmd_vel_mux/input/keyop
s=0.05
c=0
z_ang=0
p = rospy.Publisher('/yocs_cmd_vel_mux/input/keyop', Twist)
while not rospy.is_shutdown():
    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = x_speed;                   # our forward speed
    twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
    twist.angular.x = 0; twist.angular.y = 0;   #          or these!
    twist.angular.z = z_ang;                        # rotation in rad / sec

    p.publish(twist)
    rospy.loginfo("Sending Twist: [%f,%f]"%(twist.angular.z ,twist.linear.x))
    rospy.sleep(0.2) 
    c=c+s
    if (c>0.5):
	s=-0.05
    if (c<-0.5):
	s=0.05
    z_ang=c
# create a new message
twist = Twist()

# note: everything defaults to 0 in twist, if we don't fill it in, we stop!
rospy.loginfo("Stopping!")
p.publish(twist)
  
