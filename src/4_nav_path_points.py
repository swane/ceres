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
global Er,Nr
global gpsstatus
global goal_N
global goal_E
global goal_lat
global goal_lon
global lat
global lon
global head
latlist = []
lonlist = []
goal_N=0
goal_E=0

head=0
lat=0
lon=0
Nr=0
Er=0
gpsstatus=0

L=0.69

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
    global Er,Nr
    global gpsstatus
    global lat
    global lon
   
    lat=msg.latitude
    lon=msg.longitude
    c,Er,Nr = LLtoUTM(23,lat, lon) #current position, transform to cutting head position
    gpsstatus=msg.status.status

def HEADcallback(msg):
	global head
	head = float(msg.relPosHeading)/100000
	head=bearingwrap(head+90)
	

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
	global Er,Nr
        global stat
	global goal_N
	global goal_E
	global goal_lat
	global goal_lon
	global lat
	global lon
	global head
	print lat
	dist, goalbearing=dist_bearing(Nr,Er,goal_N,goal_E)
	
	aim_bear=closest_bearing_difference(head,goalbearing)
	
	steer=aim_bear*steer_K
	steer = clamp(steer, minsteer, maxsteer)
	#publish steer and speed
	steer=aim_bear
	pubStr.publish(steer)
	#move_cmd=Twist()
	speed=0
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
	# create a twist message, fill in the details
        x_speed=speed
        zang=(speed/L)*tan(-steer/57.296)
        twist = Twist()
        twist.linear.x = x_speed;                   # our forward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = z_ang;                        # rotation in rad / sec
        pubtwist.publish(twist)


def pathcallback(msg):
	#Appending positions, as many as there are
	global latlist
	global lonlist
	print "Positions incoming"
	print len(msg.poses),"positions appended"
	n=0
	while n<len(msg.poses):	
		#print 'Position:',n
		latp= msg.poses[n].pose.position.x
		lonp= msg.poses[n].pose.position.y 
		print latp
		print lonp
		latlist.append(latp)
		lonlist.append(lonp)
		n+=1
	poplist()
	
def poplist():
	global latlist
	global lonlist
	global goal_N
	global goal_E
	global goal_lat
	global goal_lon
	if (len(latlist)>0):
		goal_lat=latlist.pop(0)
		goal_lon=lonlist.pop(0)
		(z, goal_E, goal_N) = utm.LLtoUTM(23, goal_lat,goal_lon)
	else:
		goal_lat=0
		goal_lon=0
	print "Goal lat,lon:"
	print goal_lat
	print goal_lon



def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('navigate_n_points',anonymous=False)
    
sub=rospy.Subscriber('positions',Path,pathcallback,queue_size=10)
 
pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1) 
pubtwist = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist) 
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
#rospy.Subscriber("/gps/navstatus", NavSTATUS, Statuscallback)

#rospy.Subscriber("gps/navrelposned", NavRELPOSNED9, HEADcallback)
rospy.Subscriber("gps/gpsheading", NavRELPOSNED9, HEADcallback)

print "Waiting for path to be published"
	
while not rospy.is_shutdown():
        
	if (len(latlist)>0):
		seek_point()
	
	rospy.sleep(0.1)
	


