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

from nav_msgs.msg import Odometry

global lat,lon  
global head
global Er,Nr
global stat

head=0
lat=0
lon=0
Nr=0
Er=0
stat=0

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


def GPScallback(msg):
    global Er,Nr 
    global lat,lon  
    print "GPS callback"
    lat=msg.latitude
    lon=msg.longitude
    c,Er,Nr = LLtoUTM(23,lat, lon) #current position, transform to cutting head position

def HEADcallback(msg):
	global head
	head = float(msg.relPosHeading)/100000
	#print "%f" % (head)
	#orientation_q=msg.orientation
	#orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
	#(roll,pitch,yaw)=euler_from_quaternion(orientation_list)
    	#head=yaw*57.2957795
    	#if head<0:
        #	head=head+360
        #head = 360 - head

def Statuscallback(msg):
	global stat
	
	stat= msg.fixStat
	
	

def show_point():
	global Er,Nr
   
        global head	
	global lat,lon
	global stat
	print "%f, %f :latlon:  %f, %f :EN:  %f, %f" % (stat,head,lat, lon, Er, Nr)
	

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('SHOW_GPS',anonymous=False)
    
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
rospy.Subscriber("/gps/navstatus", NavSTATUS, Statuscallback)
#rospy.Subscriber("gps/navrelposned", Imu, HEADcallback)  
 
rospy.Subscriber("gps/navrelposned", NavRELPOSNED9, HEADcallback)
while not rospy.is_shutdown():
	show_point()
	rospy.sleep(0.1)


