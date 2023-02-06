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

global goal_N
global goal_E

goal_N=5848199.0
goal_E=538479.0

global head
global Er,Nr
lat=52.782337
lon=-2.429836
c,goal_E,goal_N = LLtoUTM(23,lat, lon) 
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

def carrot_point(CN,CE,N1,E1,N2,E2,Aim):

	
	AF=((N1-CN)*(N1-N2)+(E1-CE)*(E1-E2))/(math.sqrt((N1-N2)**2+(E1-E2)**2))

	AB=math.sqrt((N1-N2)**2+(E1-E2)**2)
	FN=N1+(N2-N1)*(AF/AB)
	FE=E1+(E2-E1)*(AF/AB)

	carrot_N=FN+(N2-N1)*(Aim/AB)

	carrot_E=FE+(E2-E1)*(Aim/AB)
	
	return carrot_N,carrot_E

def GPScallback(msg):
    global Er,Nr   
    lat=msg.latitude
    lon=msg.longitude
    c,Er,Nr = LLtoUTM(23,lat, lon) #current position, transform to cutting head position

def HEADcallback(msg):
	global head
	orientation_q=msg.orientation
	orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
	(roll,pitch,yaw)=euler_from_quaternion(orientation_list)
    	head=yaw*57.2957795
    	if head<0:
        	head=head+360

def seek_point():
	global Er
        global Nr
        global head	
	#Read the GPS location from the subscription
	
	dist, goalbearing=dist_bearing(Nr,Er,goal_N,goal_E)
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
    
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
rospy.Subscriber("/heading", Imu, HEADcallback)

pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)
while not rospy.is_shutdown():
	seek_point()
	rospy.sleep(0.1)


