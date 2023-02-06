#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from math import pi, sin, cos, tan, sqrt, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
import utm
import time
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus



global goal_N
global goal_E

global lat
global lon
global head
lat=0
lon=0
head=0

def call_nav(navsat):
	global lat
	global lon
	lat=navsat.latitude
	lon=navsat.longitude
	#rospy.loginfo("Latitude: %s", navsat.latitude)      #Print GPS co-or to terminal
	#rospy.loginfo("Longitude: %s", navsat.longitude)
#gpsfix_msg = GPSFix()



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
	#Read the GPS location from the subscription
	(z, e, n) = utm.LLtoUTM(23, lat,lon)
	dist, goalbearing=dist_bearing(n,e,goal_N,goal_E)
	aim_bear=closest_bearing_difference(head,goalbearing)
	print "Hello, I'm at: %f, %f, dist=%f, bearing=%f, goal bearing:%f" % (e, n, dist, head,goalbearing)
	#p = rospy.Publisher('cmd_vel', Twist)

        #z_ang=(180-bear)*0.01
        #print z_ang
        #twist = Twist()
        #twist.linear.x = 1.0;                   # our forward speed
        #twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
        #twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        #twist.angular.z = z_ang;                        # rotation in rad / sec

        #sendtwist.publish(twist)

	#publish steer and speed
	steer=aim_bear
	pubStr.publish(steer)
	#move_cmd=Twist()
	if dist>2:
		speed=1
	else:
		speed=0
	
	pubSpd.publish(speed)
	print "Publishing: %f, %f" % (speed, steer)
	
    	#move_cmd.linear.x=linear_speed
    	#move_cmd.angular.z=rotation_speed
    	#p.publish(move_cmd)

	

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('CERES_NAV_GPS',anonymous=False)
    
#rospy.Subscriber("/gps/fix", NavSatFix, call_nav)
rospy.Subscriber("/gps/fix", NavSatFix, call_nav)

#time.sleep(4)
(z, StE, StN) = utm.LLtoUTM(23, lat,lon)
print "Started at:"
print StN,StE

while not rospy.is_shutdown():
	(z, EnE, EnN) = utm.LLtoUTM(23, lat,lon)
	dist,bear=dist_bearing(StN,StE,EnN,EnE)
	print lat,lon
	print EnE,EnN
	print dist
	if dist>5:
		print "Calculated heading:"
		print bear
		StN=EnN
		StE=EnE
	rospy.sleep(1)


