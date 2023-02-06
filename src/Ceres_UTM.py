#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from math import pi, sin, cos, tan, sqrt, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
import time
import utm

global goal_N
global goal_E

global lat
global lon
global head
lat=0
lon=0
head=0


def latcallback(msg):
    global lat
    lat=msg.data
    #print lat

def loncallback(msg):
    global lon
    lon=msg.data

def headcallback(msg):
    global head
    head=msg.data

def closest_bearing_difference(current_bearing,goal_bearing):
    cl_bearing=goal_bearing-current_bearing
    if (cl_bearing>180):
	cl_bearing=(cl_bearing-360)
    if (cl_bearing<-180): 
	cl_bearing=(cl_bearing+360)
    return cl_bearing

def dist_bearing(StN,StE,EnN,EnE):
	dist=sqrt(((EnN-StN)*(EnN-StN))+((EnE-StE)*(EnE-StE)))
	bearing=atan2((EnE-StE),(EnN-StN))
	bearing=bearing*57.29578
	if bearing<0:
		bearing=360+bearing
	return (dist, bearing)


def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('CERES_UTM',anonymous=False)
    
rospy.Subscriber("/CeresLat", Float64, latcallback)
rospy.Subscriber("/CeresLon", Float64, loncallback)
rospy.Subscriber("/CeresHead", Float64, headcallback)
time.sleep(4)
(z, StE, StN) = utm.LLtoUTM(23, lat,lon)
print "Started at:"
print StN,StE

while not rospy.is_shutdown():
	(z, EnE, EnN) = utm.LLtoUTM(23, lat,lon)
	print lat, lon
	print EnN, EnE
	rospy.sleep(1)


