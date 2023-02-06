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

global L
L=0.69  #vehicle length in m

global prevod
prevod=0

global current_time
global last_time


lat=0
lon=0
bear=0
steer=0
speed=0
head=0
actspd=0
actstr=0
buzz=0

def speedcallback(msg):
   global speed
   global s
   speed=msg.data
   s.write('$%2.1f,%2.1f,%2.1f:\r\n'%(steer,speed,buzz))
   

def steercallback(msg):
    global steer
    global s
    steer=msg.data
    print "Steer command:"
    print steer
    
    print ('$%2.1f,%2.1f:\r\n'%(steer,speed))
    s.write('$%2.1f,%2.1f,%2.1f:\r\n'%(steer,speed,buzz))
   
def buzzcallback(msg):
    global buzz
    global s
    buzz=msg.data
    
    s.write('$%2.1f,%2.1f,%2.1f:\r\n'%(steer,speed,buzz))
def twistcallback(msg):
        global s
	global speed
	global steer
	global L
	global buzz
	steer= atan(msg.angular.z*L/msg.linear.x)*57.296
	speed=msg.linear.x
	print steer
	print speed
	s.write('$%2.1f,%2.1f,%2.1f:\r\n'%(steer,speed,buzz))
def read_robot(datastring):  # $odom,steerangle,act speed,lat,lonbearing
	global s
	global x
	global y
	global odo
        global vth
	global prevod
        global fwdx
	odo=0.0
	#print(datastring)
	splitstring=datastring.split(",")
	if len(splitstring)<5:
		print "Error invalid input"
		print datastring
		s.flushInput()
		s.flushOutput()
		s.flush()
	else:
		#lts,lns,heads,steers,spds=splitstring[0:5]
		od,steers,spds,lts,lns,heads=splitstring[0:6]
		try:
		
		    n=float(od)
		
		    odo=n-prevod
		    prevod=n
		    #x+=odo*0.1  #Odometry conversion
		    fwdx=n
                    #fwdx=0.1
		    #print "x:"
		    #print(x)
			
		except:
		    odo=0.0
		    fwdx=0.0
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
			print head
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
		#calc thetadot from actspd and actstr (degrees, positive to the right)
                
		vth=(actspd/L)*tan(-actstr/57.296)

		#print rospy.Time.now()
		print rospy.get_time()
		print "Location:"
		print(lat)
		print(lon)
		
		#y+=0 #steer to y conversion
		
		#set_odom()
	#print "Returning"
def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	s.close()
ports=list(serial.tools.list_ports.comports())
port=""
for p in ports:
	#print p[2]
	if "USB" in p[2]:
		port=p[0]
		
		#print "Found!"
#arduino_ports=[
#	p.device
#	for p in serial.tools.list_ports.comports()
#	print p
	#if 'Arduino' in p.description
#]
#if not arduino_ports:
#	raise IOError("No Arduino found")



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
rospy.init_node('test_Ceres_ROS',anonymous=False)
current_time = rospy.Time.now()
last_time = rospy.Time.now()
time.sleep(1)
s.flushInput()
s.flushOutput()
publat=rospy.Publisher("/CeresLat", Float64, queue_size=1)
publon=rospy.Publisher("/CeresLon", Float64, queue_size=1)
pubhead=rospy.Publisher("/CeresHead", Float64, queue_size=1)
pubAspd=rospy.Publisher("/CeresActSpd", Float64, queue_size=1)
pubAstr=rospy.Publisher("/CeresActStr", Float64, queue_size=1)


rospy.Subscriber("/CeresSpeed", Float64, speedcallback)
rospy.Subscriber("/CeresSteer", Float64, steercallback)
rospy.Subscriber("/cmd_vel", Twist, twistcallback)
rospy.Subscriber("/CeresBuzz", Float64, buzzcallback)
   

 
rate = rospy.Rate(10) # 10hz
print "Running on Arduino"
    # spin() simply keeps python from exiting until this node is stopped
while not rospy.is_shutdown():
	#print "About to read"
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
	#try:
		#s.write('$%2.1f,%2.1f:'%(steer,speed))
		#s.flush()
	#except serial.SerialException as e:
		#return none
	#	pass
		
		#return none
	
	#print "Sending"
	#print steer
	#print "Read"
	#print(st)
	
	#print lat

	#rospy.sleep(0.1)
	#rospy.spin()

            


