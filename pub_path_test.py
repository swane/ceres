#!/usr/bin/env python
#https://roboticsbackend.com/ros-topic-command-line-tools-practical-example-rostopic-and-rosmsg/
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('publish_path')
pub=rospy.Publisher('positions',Path,queue_size=10,latch=True)

rate=rospy.Rate(2)
msg = Path()
msg.header.frame_id = "map"
msg.header.stamp = rospy.Time.now()

lat=52.767325
lon=-2.123456
count=0
while count<5:
	
	pose = PoseStamped()
        pose.pose.position.x = lat
        pose.pose.position.y = lon
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        msg.poses.append(pose)
	print lat
	print lon
	print count
	lat+=0.2
	lon+=0.55
	count+=1
	rate.sleep()
print "Publishing now"
pub.publish(msg)
        