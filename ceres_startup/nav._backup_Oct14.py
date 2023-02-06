#!/usr/bin/env python

# Matt Butler and Sam Wane. Harper Adams University 2022.
# recieves mission as a  list of lists on argv[1], and a starting point on argv[2].
# to test from a terminal:
# python nav.py '[["-3.0953963", "52.4604236", "start"], ["-1.9692977", "53.0455683", "middle"], ["-1.0519393", "52.5941015", "end"]]' 0
# requires Python2 and ROS environment

import socket # to send out status updates (and/or recieve commnds)
import datetime

import sys
import json
import time
from math import pi, sin, cos, tan, sqrt, atan2

import rospy
from std_msgs.msg import String
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

# Create a UDP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address = ('', 1963)
s.bind(server_address)
print("Recieving on UDP port 1963")

# Enable broadcasting mode
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# function to send a string message
def sendUDPinfo(msg):
    b = msg.encode('utf-8')
    s.sendto(b, ("<broadcast>", 1964))

# example function call
now = datetime.datetime.now()
sendUDPinfo(str(now))

# function to send a target message to UI
def sendUDPtarget(msg):
    msg = str(msg)
    b = msg.encode('utf-8')
    s.sendto(b, ("<broadcast>", 1965))
    
sendUDPtarget(350)
    
# function to send a heading message to UI
def sendUDPheading(msg):
    msg = str(msg)
    b = msg.encode('utf-8')
    s.sendto(b, ("<broadcast>", 1966))
    
sendUDPheading(10)
    
# function to send a steering message to UI
def sendUDPsteering(msg):
    msg = str(msg)
    b = msg.encode('utf-8')
    s.sendto(b, ("<broadcast>", 1967))
    
sendUDPsteering(-10)

## load passed in parameters (and check them)

try:
    mission = sys.argv[1]
except: # default values for testing. WARNING: Thonny uses Python 3!
    #mission = '[["-3.0953963", "52.4604236", "start"], ["-1.9692977", "53.0455683", "middle"], ["-1.0519393", "52.5941015", "end"]]'
    print('No mission argument given - quitting')
    time.sleep(3)
    sys.exit()

try:
    start_point = sys.argv[2]
    print("Starting from point " + str(int(start_point)+ 1))
    sendUDPinfo("Starting from point " + str(int(start_point)+ 1))
except: # default values for testing. WARNING: Thonny uses Python 3!!!!!!!!!
    print('No start point provided - start at beginning')
    sendUDPinfo('No start point provided - start at beginning')
    start_point = "0"

try: # use Python dictionry
    m_dict = json.loads(mission)
    maxpos=len(m_dict)
except:
    print('Mission is malformed - quitting')
    sendUDPinfo('Mission is malformed - quitting')
    time.sleep(6)
    sys.exit()

time.sleep(4) # just for reading messages during development

## Navigate through mission from start point


''' not needed in Python - declare as globlal in functions
global lasttime
global integError 
global lastError
global lat,lon  
global head
global Er,Nr
global stat
global pos
global maxpos
global goal_lat
global goal_lon
'''



latlist = []
lonlist = []


lasttime=0.0
interError=0.0
lastError=0.0

head=0
lat=0
lon=0
Nr=0
Er=0
stat=0

maxsteer=30.0
minsteer=-30.0
Ks=1
Kis=0.1
Kds=1

goal_pos=0





#(z, goal_E, goal_N) = utm.LLtoUTM(23, latlist[0],lonlist[0])



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


def Output_PID(Kp,  Ki,  Kd,  error,  maxerror):
        s=rospy.Time.now().to_sec()
        dt=s-begin      
        begin=s  
        if ((error > -maxerror) and (error < maxerror)): integError += error * dt
        compP = error * Kp
        compI = integError * Ki
        compD = ((error - lastError) / dt) * Kd
        lastError = error
        
        return compP + compI + compD


      

def seek_point():
    
    dist, goalbearing=dist_bearing(Nr,Er,goal_N,goal_E)
    
    aim_bear=closest_bearing_difference(head,goalbearing)
    
    steer=aim_bear*1
    #steer=Output_PID(Ks,  Kis,  Kds,  aim_bear,  20)
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
    global pos
    global latlist
    global lonlist
    global goal_E
    global goal_N
    pos=pos+1
    if (len(latlist)>0):
        lat=latlist.pop(0)
        lon=lonlist.pop(0)
        (z, goal_E, goal_N) = utm.LLtoUTM(23, lat,lon)
    else:
        lat=0
        lon=0
    
#sub=rospy.Subscriber('positions',Path,pathcallback,queue_size=10)

def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    self.cmd_vel.publish(Twist())

rospy.init_node('navigate_node_red_points',anonymous=False)
    
 
pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)  
rospy.Subscriber("/gps/fix", NavSatFix, GPScallback)
rospy.Subscriber("/gps/navstatus", NavSTATUS, Statuscallback)

rospy.Subscriber("gps/navrelposned", NavRELPOSNED9, HEADcallback)
begin=rospy.Time.now().to_sec()



pos=0
for i in range(int(start_point), len(m_dict)):

    print(m_dict[i][1]) # lat
    print(m_dict[i][0]) # lon
    print(m_dict[i][2]) # info
    lat=float(m_dict[i][1])
    lon=float(m_dict[i][0])
    latlist.append(lat)
    lonlist.append(lon)    
    
poplist()
(z, goal_E, goal_N) = utm.LLtoUTM(23, lat,lon)

print()
print("The route has " + str(len(m_dict)) + " points")


sys.stderr.write(str(mission) + ' ' + start_point)

while (pos<=maxpos):
        #show_point()
    seek_point()
    rospy.sleep(0.1)




