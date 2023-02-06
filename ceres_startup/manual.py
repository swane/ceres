#!/usr/bin/env python
# Matt Butler & Sam Wane, Harper Adams University 2022
# Listen for manual speed and steer commands over UDP


import socket
import json
from math import tan,pi

from threading import Thread

# ROS imports
import rospy
#from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# Create dedicated UDP sockets
sp_UDP_IP = ""
sp_UDP_PORT = 1971
sp_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sp_sock.bind((sp_UDP_IP, sp_UDP_PORT))
sp_sock.settimeout(0.8)
print("Recieving speed on UDP port 1971") # listening for speed

str_UDP_IP = ""
str_UDP_PORT = 1972
str_sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
str_sock.bind((str_UDP_IP, str_UDP_PORT))
str_sock.settimeout(0.8)
print("Recieving steering on UDP port 1972") # listening for steering

#Initialise ROS Publishers
#pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
#pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)
cmdman = rospy.Publisher('/cmd_vel_mux/cmd_vel_man', Twist, queue_size=1)
speed = 0
steer = 0
#Vehicle kinematics
L=0.69
def send_twist():
    global cmdman
    global steer
    global speed
    steerrad=float(steer)/180*pi
    x_speed=float(speed)
    zdot=(x_speed*tan(-steerrad))/L
    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = x_speed                   # our forward speed
    twist.linear.y = 0; twist.linear.z = 0    # we can't use these!        
    twist.angular.x = 0; twist.angular.y = 0   #          or these!
    twist.angular.z = zdot                       # rotation in rad / sec   
    cmdman.publish(twist)

def spstr_function(arg): # recieve speed from (for example) NodeRed
    global speed
    global steer
    print("Speed steer function running")
    while 1:
            
            #print(len(data))
            try:
                data, address = sp_sock.recvfrom(4096)
                b = data.decode('utf-8')
                d = json.loads(b)
                command = d['command']
                print('Encoded command = ' + str(command))

                # received over UDP on port 1971

                steer = (command & 2016)  // 32
                

                speed = command & 31
		speed=speed-10
		speed=speed/10
		steer=steer-30
		print('received speed = ' + str(speed))
		print('received steer = ' + str(steer))
                send_twist()
                ###########################
                
                # Sam code to publish
                
                ###########################
            except:
                print("Bad/no speed data from host")
            #pubSpd.publish(speed)
            

'''
def sp_function(arg): # recieve speed from (for example) NodeRed
    global speed
    print("Speed function running")
    while 1:
            
            #print(len(data))
            try:
                data, address = sp_sock.recvfrom(4096)
                b = data.decode('utf-8')
                d = json.loads(b)
                #print(b)
                speed = d['speed']
                print('Speed: ' + str(speed))
                
                ###########################
                
                # Sam code to publish
                
                ###########################
            except:
                print("Bad/no speed data from host")
            #pubSpd.publish(speed)
            send_twist()


def str_function(arg): # recieve speed from (for example) NodeRed
    global steer
    print("Steer function running")
    while 1:
            
            ##print(len(data))
            try:
                data, address = str_sock.recvfrom(4096)
                b = data.decode('utf-8')
                d = json.loads(b)
                #print(b)        
                steer = d['steer']
                print('Steer: ' + str(steer))
                
                ###########################
                
                # Sam code to publish
                
                ###########################
            except:
                print("Bad/no steer data from host")
            #pubStr.publish(steer)
            send_twist()
'''

if __name__ == '__main__':
    #sp_thread = Thread(target = sp_function, args = (10, ))
    #str_thread = Thread(target = str_function, args = (10, ))
    #sp_thread.start()
    #str_thread.start()
    spstr_thread = Thread(target = spstr_function, args = (10, ))
    spstr_thread.start()
    #print("Thread running")
    
    ###########################
                
    # Sam code to publish
                
    ###########################
    rospy.init_node('Manual_Socket_Control',anonymous=False)
 
    while(1):
        pass
