#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from math import pi, sin, cos, tan, sqrt, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from latlongtoutm import *
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry


global head
global Er,Nr

head=0

def bearingwrap(b):
    if (b>=360):
        b=b-360
    if (b<=0):
        b=b+360
    return b

def bear_to_deg(b):
    deg=450-b
    deg=bearingwrap(deg)
    return deg

def deg_to_bear(d):
    b=450-d
    b=bearingwrap(b)
    return b

def bearing_to(ce,cn,ge,gn):
    bear=math.atan2(ge-ce,gn-cn)*57.2957795
    if (bear<0):
        bear=bear+360
    return bear



def GPScallback(msg):
    global Er,Nr,head   
    lat=msg.latitude
    lon=msg.longitude
    c,Er,Nr = LLtoUTM(23,lat, lon) #current position
    print "E: %f, N: %f, Head: %f, bearing=%f, goal bearing:%f" % (Er, Nr, head, head,goalbearing)

def HEADcallback(msg):
	global head
	orientation_q=msg.orientation
	orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
	(roll,pitch,yaw)=euler_from_quaternion(orientation_list)
    	head=yaw*57.2957795
    	if head<0:
        	head=head+360


	

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())
#Requires roslaunch ublox_gps ardusimple.launch
rospy.init_node('CERES_SHOW_GPS',anonymous=False)
    
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
rospy.Subscriber("/heading", Imu, HEADcallback)

while not rospy.is_shutdown():
	seek_point()
	rospy.sleep(0.1)


