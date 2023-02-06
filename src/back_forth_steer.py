#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from math import pi, sin, cos, tan, sqrt, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int16


	
	

def shutdown(self):
	rospy.loginfo("Stopping the robot...")
	self.cmd_vel.publish(Twist())

rospy.init_node('BACK_FORTH_STEER',anonymous=False)
  


pubStr=rospy.Publisher("/CeresSteer", Float64, queue_size=1)
pubSpd=rospy.Publisher("/CeresSpeed", Float64, queue_size=1)
print "Starting"
while not rospy.is_shutdown():
	pubStr.publish(10)
	print "Steer 10"
	rospy.sleep(2)
	print "Steer 0"
	pubStr.publish(0);
	rospy.sleep(2)


