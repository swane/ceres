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

global lasttime=0.0
global integError = 0.0
global lastError=0.0

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

L=0.69

maxsteer=30.0
minsteer=-30.0
Ks=1
Kis=0.1
Kds=1

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

(z, goal_E, goal_N) = utm.LLtoUTM(23, latlist[0],lonlist[0])
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


def Output_PID(Kp,  Ki,  Kd,  error,  maxerror)
        s=rospy.Time.now().to_sec()
	dt=s-begin      
	begin=s  
        if ((error > -maxerror) && (error < maxerror)): integError += error * dt
        compP = error * Kp
        compI = integError * Ki
        compD = ((error - lastError) / dt) * Kd
        lastError = error
        
        return compP + compI + compD;
}

      

def seek_point():
	
	dist, goalbearing=dist_bearing(Nr,Er,goal_N,goal_E)
	
	aim_bear=closest_bearing_difference(head,goalbearing)
	
	#steer=aim_bear*steer_K
	steer=Output_PID(Ks,  Kis,  Kds,  aim_bear,  20)
	steer = clamp(steer, minsteer, maxsteer)
	#publish steer and speed
	#steer=aim_bear
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
	

rospy.init_node('subscribe_path')
sub=rospy.Subscriber('positions',Path,pathcallback,queue_size=10)

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('navigate_n_points',anonymous=False)
    
 
pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)
pubtwist = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist)
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
rospy.Subscriber("/gps/navstatus", NavSTATUS, Statuscallback)

rospy.Subscriber("gps/navrelposned", NavRELPOSNED9, HEADcallback)
begin=rospy.Time.now().to_sec()
poplist()
if (lat==0):
	print "Waiting for path to be published"
	while lat==0:
		poplist()

while not rospy.is_shutdown():
        #show_point()
	seek_point()
	rospy.sleep(0.1)

PID
begin=rospy.Time.now().to_sec()   

while not rospy.is_shutdown():
	s=rospy.Time.now().to_sec()
	t=s-begin      
	print t
        begin=s  

