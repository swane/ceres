#!/usr/bin/env python

#Connects to the Arduino Com port, listens for CeresSpeed, CeresSteer topics and sends its lat and lon as CeresLat, CeresLon, CeresHead
#version 2 to handle recognising Arduino, and exceptions
import rospy
import serial
import warnings
import serial.tools.list_ports
import time
from std_msgs.msg import String
from math import pi, sin, cos, tan, atan, sqrt, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from std_msgs.msg import Bool
import utm
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



global x
global y
global th
global vx
global vy
global vth

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = -0.0
vth = 0.0



global s
global lat
global lon
global head
global actspd
global actstr
global head
global speed
global steer
global fwdx
fwdx=0.0

L=0.69  #vehicle length in m


lat=0
lon=0
bear=0
steer=0
speed=0
head=0
actspd=0
actstr=0
buzz=0
squirt=0

newdatatoUSB=False

def speedcallback(msg):
   global speed
  
   speed=msg.data
   newdatatoUSB=True
   

def steercallback(msg):
    global steer
    steer=msg.data
    newdatatoUSB=True
   
def buzzcallback(msg):
    global buzz
    buzz=msg.data
    newdatatoUSB=True
    
def squirtcallback(msg):
    global squirt
    squirt=msg.data
    newdatatoUSB=True
    
def twistcallback(msg):
	global speed
	global steer
	steer= atan(msg.angular.z*L/msg.linear.x)*57.296
	speed=msg.linear.x
	newdatatoUSB=True
    
def read_robot(datastring):  # $odom,steerangle,act speed,lat,lonbearing
	global s
	global x
	global y
	global odo
        global vth
	global prevod
        global fwdx

	#print(datastring)
	splitstring=datastring.split(",")
	if len(splitstring)<7:
		print "Error invalid input"
		print datastring
		s.flushInput()
		s.flushOutput()
		s.flush()
	else:
		#lts,lns,heads,steers,spds=splitstring[0:5]
            odos,steers,spds,lts,lns,heads,batts,errors=splitstring[0:8]
            try:
                odo=float(odos)
                pubOdo.publish(odo)
            except:
                odo=0
            try:
                lat=float(lts)
                publat.publish(lat)
            except:
                lat=0
            try:
                lon=float(lns)
                publon.publish(lon)
            except:
                lon=0
            try:
                head=float(heads)
                pubhead.publish(head)
            except:
                head=0
            try:
                actspd=float(spds)
                pubAspd.publish(actspd) 
            except:
                actspd=0
            try:
                actstr=float(steers)
                pubAstr.publish(actstr)
            except:
                actstr=0
            try:
                batt=float(batts)
                pubBatt.publish(batt)
            except:
                batt=0
            try:
                error=float(errors)
                pubError.publish(errors)
            except:
                errors=0
 
		#calc thetadot from actspd and actstr (degrees, positive to the right)
                
		vth=(actspd/L)*tan(-actstr/57.296)


def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	s.close()

port = '/dev/ttyARD'
#port=arduino_ports[0]
print "Opening on port"

print port
connect_tries = 1
portOK = False

while not portOK:

    try: 
        s = serial.Serial(port,57600,timeout=1,writeTimeout=0)
        portOK = True
    except:
        print "Port " + port + " not connected  - " + str(connect_tries) + " tries"
        time.sleep(2)
        connect_tries = connect_tries + 1
        
s.close() 
s.open()
s.flush()
rospy.init_node('Arduino_Ceres_ROS',anonymous=False)

time.sleep(1)
s.flushInput()
s.flushOutput()
publat=rospy.Publisher("/CeresLat", Float64, queue_size=1)
publon=rospy.Publisher("/CeresLon", Float64, queue_size=1)
pubhead=rospy.Publisher("/CeresHead", Float64, queue_size=1)
pubAspd=rospy.Publisher("/CeresActSpd", Float64, queue_size=1)
pubAstr=rospy.Publisher("/CeresActStr", Float64, queue_size=1)
pubBatt=rospy.Publisher("/CeresBatt", Int8, queue_size=1)
pubError=rospy.Publisher("/CeresError", Int8, queue_size=1)
pubOdo=rospy.Publisher("/CeresOdo", Int16, queue_size=1)

rospy.Subscriber("/CeresSpeed", Float64, speedcallback)
rospy.Subscriber("/CeresSteer", Float64, steercallback)
rospy.Subscriber("/cmd_vel", Twist, twistcallback)
rospy.Subscriber("/CeresBuzz", Bool, buzzcallback)
rospy.Subscriber("/CeresSquirt", Int8, squirtcallback)   

 
rate = rospy.Rate(10) # 10hz
print "Running on Arduino"
    # spin() simply keeps python from exiting until this node is stopped
while not rospy.is_shutdown():

	c=""
        try:
            c=s.read(1)
        except serial.SerialException as e:
            pass
        if c=="$":
	    try:
		st=s.readline()
		#print st
	    except serial.SerialException as e:
		#return none
		pass
		
		#return none
	    
	    read_robot(st)
	s.flushInput()
	s.flushOutput()
	s.flush()
	if newdatatoUSB==True:
            s.write('$%2.1f,%2.1f,%d,%d:\r\n'%(steer,speed,buzz,squirt))
            newdatatoUSB=False

            


