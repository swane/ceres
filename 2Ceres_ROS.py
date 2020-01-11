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


global s
global lat
global lon
global head
global actspd
global actstr
global head
global speed
global steer

lat=0
lon=0
bear=0
steer=0
speed=0
head=0
actspd=0
actstr=0


def speedcallback(msg):
   global speed
   speed=msg.data
   #s.write('$%2.1f,%2.1f:\r\n'%(steer,speed))
   

def steercallback(msg):
    global steer
    #print "Steer OK"
    steer=msg.data
    #print steer
    #s.write('$%2.1f,%2.1f:\r\n'%(steer,speed))
   

def twistcallback(msg):
	global speed
	global steer
	L=0.69
	steer= atan(msg.angular.z*L/msg.linear.x)*57.296
	speed=msg.linear.x
	print steer
	print speed
	#s.write('$%2.1f,%2.1f:\r\n'%(steer,speed))

def read_robot(datastring):
	global s
	#print(datastring)
	splitstring=datastring.split(",")
	if len(splitstring)<5:
		print "Error invalid input"
		print datastring
		s.flushInput()
		s.flushOutput()
		s.flush()
	else:
		lts,lns,heads,steers,spds=splitstring[0:5]
		#print(lts)
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
		(z, e, n) = utm.LLtoUTM(23, lat,lon)
		pubN.publish(n)
		pubE.publish(e)
		
		print "Location:"
		print(lat)
		print(lon)
		print(n)
		print(e)
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
#port = '/dev/ttyACM0'
#port=arduino_ports[0]
print "Opening on port"
print port
s = serial.Serial(port,57600,timeout=1,writeTimeout=0)
s.close()
s.open()
s.flush()
rospy.init_node('test_Ceres_ROS',anonymous=False)
time.sleep(4)
s.flushInput()
s.flushOutput()
publat=rospy.Publisher("/CeresLat", Float64, queue_size=1)
publon=rospy.Publisher("/CeresLon", Float64, queue_size=1)
pubN=rospy.Publisher("/CeresN", Float64, queue_size=1)
pubE=rospy.Publisher("/CeresE", Float64, queue_size=1)
pubhead=rospy.Publisher("/CeresHead", Float64, queue_size=1)
pubAspd=rospy.Publisher("/CeresActSpd", Float64, queue_size=1)
pubAstr=rospy.Publisher("/CeresActStr", Float64, queue_size=1)

rospy.Subscriber("/CeresSpeed", Float64, speedcallback)
rospy.Subscriber("/CeresSteer", Float64, steercallback)
rospy.Subscriber("/cmd_vel", Twist, twistcallback)
   

 
rate = rospy.Rate(10) # 10hz

    # spin() simply keeps python from exiting until this node is stopped
while not rospy.is_shutdown():
	#print "About to read"
	c=""
	while (c!='$'):
		#print c
		c=s.read(1)
	try:
		st=s.readline()
		#print st
	except serial.SerialException as e:
		#return none
		pass
		#return none
	else:
		read_robot(st)
	s.flushInput()
	s.flushOutput()
	s.flush()
	try:
		s.write('$%2.1f,%2.1f:'%(steer,speed))
		s.flush()
	except serial.SerialException as e:
		#return none
		pass
		#return none
	
	#print "Sending"
	#print steer
	#print "Read"
	#print(st)
	
	#print lat

	rospy.sleep(0.1)
	#rospy.spin()

            


