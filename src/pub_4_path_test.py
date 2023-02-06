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

lat=52.782178
lon=-2.429445
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
lat=52.782438
lon=-2.429827
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

lat=52.782357
lon=-2.429999
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

lat=52.782273
lon=-2.429882
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

lat=52.782178
lon=-2.429445
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
print "Publishing now"
pub.publish(msg)
        
