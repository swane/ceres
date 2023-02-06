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
latlist = []
lonlist = []



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



maxsteer=30.0
minsteer=-30.0
steer_K=0.7

goal_pos=0

#fence near bins
#52.7821558,-2.4305712
#top corner
#52.7824158,-2.4309332
#near light
#52.7823398,-2.4311182
#closest to waste cover
#52.7820896,-2.4295769

global goal_lat
global goal_lon

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


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
    lat=msg.latitude
    lon=msg.longitude
    c,Er,Nr = LLtoUTM(23,lat, lon) #current position, transform to cutting head position

def HEADcallback(msg):
	head = float(msg.relPosHeading)/100000
	head=bearingwrap(head+90)
	

def Statuscallback(msg):
	global stat
	
	stat= msg.fixStat
	
	

def bearing_to(ce,cn,ge,gn):
    bear=math.atan2(ge-ce,gn-cn)*57.2957795
    if (bear<0):
        bear=bear+360
    return bear



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




def seek_point():
	
	dist, goalbearing=dist_bearing(Nr,Er,goal_N,goal_E)
	
	aim_bear=closest_bearing_difference(head,goalbearing)
	
	steer=aim_bear*steer_K
	steer = clamp(steer, minsteer, maxsteer)
	#publish steer and speed
	steer=aim_bear
	pubStr.publish(steer)
	#move_cmd=Twist()
	if dist>2:
		speed=0.7
	else:
		poplist()
	pubSpd.publish(speed)
	#print "Publishing: %f, %f" % (speed, steer)
	print "%f, %f :latlon:  %f, %f :EN:  %f, %f, goal:%f, aim: %f, dist:%f" % (stat,head,lat, lon, Er, Nr,goalbearing,aim_bear,dist)
    	#move_cmd.linear.x=linear_speed
    	#move_cmd.angular.z=rotation_speed
    	#p.publish(move_cmd)


def pathcallback(msg):
	#Appending positions, as many as there are
	print "Positions incoming"
	print len(msg.poses),"positions appended"
	n=0
	while n<len(msg.poses):	
		#print 'Position:',n
		lat= msg.poses[n].pose.position.x
		lon= msg.poses[n].pose.position.y 
		print lat
		print lon
		latlist.append(lat)
		lonlist.append(lon)
		n+=1
	poplist()
def poplist():
	if (len(latlist)>0):
		lat=latlist.pop(0)
		lon=lonlist.pop(0)
		(z, goal_E, goal_N) = utm.LLtoUTM(23, lat,lon)
	else:
		lat=0
		lon=0
	



def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('test_pub_steer',anonymous=False)
   
 
pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)  


c=0
s=1
while not rospy.is_shutdown():
        pubStr.publish(c)
	print c
	c=c+s
	if (c>30):
		s=-1
	if (c<-30):
		s=1
	rospy.sleep(0.1)


