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

from fixposition_driver.msg import Speed

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
global actspd
global actstr
global velocity
global steer
global fwdx
fwdx=0.0

L=0.69  #vehicle length in m

steer=0
velocity=0

actspd=0
actstr=0
buzz=0
squirt=0

newdatatoUSB=False

def velocitycallback(msg):
   global velocity
   global newdatatoUSB
   velocity=msg.data
   newdatatoUSB=True
   

def steercallback(msg):
    global steer
    global newdatatoUSB
    steer=msg.data
    newdatatoUSB=True
   
def buzzcallback(msg):
    global buzz
    global newdatatoUSB
    buzz=msg.data
    newdatatoUSB=True
    
def squirtcallback(msg):
    global squirt
    global newdatatoUSB
    squirt=msg.data
    newdatatoUSB=True
    
def twistcallback(msg):
	global velocity
	global steer
        global newdatatoUSB
	steer= atan(-msg.angular.z*L/msg.linear.x)*57.296
	velocity=msg.linear.x
	newdatatoUSB=True
    
def read_robot(datastring):  # $odom,steerangle,act velocity,lat,lonbearing
	global s
	global x
	global y
	global odo
        global vth
	global prevod
        global fwdx

	#print(datastring)
	splitstring=datastring.split(",")
	if len(splitstring)<5:
		print "Error invalid input"
		print datastring
		s.flushInput()
		s.flushOutput()
		s.flush()
	else:
		
            odos,steers,spds,batts,errors=splitstring[0:5]
            try:
                odo=float(odos)
                pubOdo.publish(odo)
            except:
                odo=0
            
            try:
                actspd=float(spds)
                pubAspd.publish(actspd)
                speeds=[0] # can be for each wheel, so is  list
                speeds[0] = int(actspd*1000)
                fixposition_speed.publish(speeds)
                #print(speeds[0])

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
pubAspd=rospy.Publisher("/CeresActSpd", Float64, queue_size=1)
pubAstr=rospy.Publisher("/CeresActStr", Float64, queue_size=1)
pubBatt=rospy.Publisher("/CeresBatt", Int8, queue_size=1)
pubError=rospy.Publisher("/CeresError", Int8, queue_size=1)
pubOdo=rospy.Publisher("/CeresOdo", Int16, queue_size=1)

fixposition_speed=rospy.Publisher("/fixposition/speed", Speed, queue_size=1)

#rospy.Subscriber("/CeresSpeed", Float64, velocitycallback)
#rospy.Subscriber("/CeresSteer", Float64, steercallback)
rospy.Subscriber("/yocs_cmd_vel/output/cmd_vel", Twist, twistcallback)
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
	    print "Sending"
	    print ('$%2.1f,%2.1f,%d,%d:\r\n'%(steer,velocity,buzz,squirt))
            s.write('$%2.1f,%2.1f,%d,%d:\r\n'%(steer,velocity,buzz,squirt))
            newdatatoUSB=False

            


