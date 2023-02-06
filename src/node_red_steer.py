#! /usr/bin/env python3

import rospy
import socket
import json
from std_msgs.msg import Float64
from threading import Thread
from time import sleep


# Create a UDP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address = ('', 1971)
s.bind(server_address)
print("Recieving on UDP port 1971")

# Enable broadcasting mode
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# create empty dictionary
zone_status = {}
zone = "stop"


''' You were broadcasting to yourself - lol. The hazards of cut'n'paste!

def send_zone(arg): # update dictionary, encode as JSON and send via UDP (to a single host or broadcast)
    global zone_status, zone
    while 1:
        
        zone_status["zone"] = zone # clear, slow, stop
    
        b = json.dumps(zone_status).encode('utf-8')
    
        #sock.sendto(b, ("127.0.0.1", 1234))
        s.sendto(b, ("<broadcast>", 1971))
        sleep(0.1)
'''

def threaded_function(arg):
    global pos, abs_steer
    while 1:
            data, address = s.recvfrom(4096)
            #print(len(data))
            b = data.decode('utf-8')
            print(b)
            d = json.loads(b)
            
            print("steering_angle: " + str(d['steer']))
            steer_topic=d['steer']
            pubStr.publish(steer_topic)
	

    
if __name__ == '__main__':
    rospy.init_node('NodeRedSteer',anonymous=False)
    pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
    thread = Thread(target = threaded_function, args = (10, ))
    ## send_thread = Thread(target = send_zone, args = (10, ))
    thread.start()
    ## send_thread.start() 
    while not rospy.is_shutdown():
        rospy.sleep(0.1)



