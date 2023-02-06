#!/usr/bin/env python


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

global goal_state
goal_state=0
global goal_N
global goal_E

goal_N=5848168.0
goal_E=538498.0



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
    #print "GPS callback"
    lat=msg.latitude
    lon=msg.longitude
    c,Er,Nr = LLtoUTM(23,lat, lon) #current position, transform to cutting head position

def HEADcallback(msg):
	global head
	head = float(msg.relPosHeading)/100000
	head=bearingwrap(head+90)
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
	global lat
        global lon
        global head
        global goal_N,goal_E	
	global goal_state
	#Read the GPS location from the subscription
	(z, e, n) = utm.LLtoUTM(23, lat,lon)
	#pubN.publish(n)
	#pubE.publish(e)
	dist, goalbearing=dist_bearing(n,e,goal_N,goal_E)
	#if goalbearing>180:
	#	goalbearing=goalbearing-180
	aim_bear=closest_bearing_difference(head,goalbearing)
	
	#print "Hello, I'm at: %f, %f, dist=%f, bearing=%f, goal bearing:%f" % (e, n, dist, head,goalbearing)
	

	#publish steer and speed
	steer=aim_bear
	pubStr.publish(steer)
	#move_cmd=Twist()
	speed=0.0
	if dist>2:
		speed=0.5
	else:
		if goal_state<1:
			goal_state=1
			goal_N=5848180.0
			goal_E=538486.0
		else:
			goal_state=0
			goal_N=5848168.0
			goal_E=538498.0
	
	pubSpd.publish(speed)
	#print "Publishing: %f, %f" % (speed, steer)
	print "%f, %f :latlon:  %f, %f :EN:  %f, %f, goal:%f, aim: %f, dist:%f" % (stat,head,lat, lon, Er, Nr,goalbearing,aim_bear,dist)
    	#move_cmd.linear.x=linear_speed
    	#move_cmd.angular.z=rotation_speed
    	#p.publish(move_cmd)


def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('navigatertk',anonymous=False)
    
 
pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)  
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
rospy.Subscriber("/gps/navstatus", NavSTATUS, Statuscallback)

rospy.Subscriber("gps/navrelposned", NavRELPOSNED9, HEADcallback)


while not rospy.is_shutdown():
        #show_point()
	seek_point()
	rospy.sleep(0.1)


