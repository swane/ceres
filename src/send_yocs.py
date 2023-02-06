#!/usr/bin/env python

""" Example code of how to move a robot forward for 3 seconds. """

# We always import roslib, and load the manifest to handle dependencies
import roslib
# roslib.load_manifest('mini_max_tutorials')
import rospy

# recall: robots generally take base movement commands on a topic 
#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist

x_speed = 1.0  # 0.1 m/s
z_ang=0.577  #20 deg / sec
# this quick check means that the following code runs ONLY if this is the 
# main file -- if we "import move" in another file, this code will not execute.
if __name__=="__main__":

    # first thing, init a node!
    rospy.init_node('move')

    # publish to /yocs_cmd_vel_mux/input/keyop
    p = rospy.Publisher('/yocs_cmd_vel_mux/input/keyop', Twist)

    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = x_speed;                   # our forward speed
    twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
    twist.angular.x = 0; twist.angular.y = 0;   #          or these!
    twist.angular.z = z_ang;                        # rotation in rad / sec

    # announce move, and publish the message
    rospy.loginfo("About to be moving forward!")
    for i in range(30):
        p.publish(twist)
	rospy.loginfo("Sending Twist: [%f,%f]"%(twist.angular.z ,twist.linear.x))
        rospy.sleep(1) 

    # create a new message
    twist = Twist()

    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    rospy.loginfo("Stopping!")
    p.publish(twist)

