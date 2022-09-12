#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 10:44:03 2022

@author: 00751570


Convert Google MyMaps kml file into points or arrays of points for Sam's robots to follow


Updated to include lineStrings 12/09/2022

"""

import xml.etree.ElementTree as ET
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('publish_path')
pub=rospy.Publisher('positions',Path,queue_size=10,latch=True)

rate=rospy.Rate(2)
msg = Path()
msg.header.frame_id = "map"
msg.header.stamp = rospy.Time.now()

tree = ET.parse('path_to_4G.kml') # change to suit file movement
root = tree.getroot()


for placecemark in root.iter('{http://www.opengis.net/kml/2.2}Placemark'):
    placemark_name = placecemark[0].text

    print()
    print(placemark_name)
    movement = placecemark[2].tag
    #print(movement)
    
    if 'Point' in movement:
        print('Point: ')
        #print(placecemark[2].tag)
        #print(placecemark[2][0].text)
        point = (placecemark[2][0].text).strip().replace(' ', '').replace(',0', '')
        point_as_list = list(filter(bool, point.splitlines()))
        
        print(point_as_list)
        #print()

    if 'LineString' in movement:
        print('Line string: ')
        #print(placecemark[2].tag)
        coords_array = (placecemark[2][1].text).strip().replace(' ', '').replace(',0', '')
        coord_list = list(filter(bool, coords_array.splitlines()))
        
        coarr = []

        for item in coord_list:
            item = item.split(',')
            coarr.append(item)
            print(item[1]) #lat
            print(item[0]) #lon
            pose = PoseStamped()
            pose.pose.position.x =float(item[1])
            pose.pose.position.y = float(item[0])
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            msg.poses.append(pose)
	    rate.sleep()
        print "Publishing now"
        pub.publish(msg)
        
        
    if 'Polygon' in movement:
        print('Polygon: ')
        #print(placecemark[2].tag)
        coords_array = (placecemark[2][0][0][1].text).strip().replace(' ', '').replace(',0', '')
        coord_list = list(filter(bool, coords_array.splitlines()))
        
        coarr = []

        for item in coord_list:
            item = item.split(',')
            coarr.append(item)
            #print(item)
        
        print(coarr)
    
# to do: put text into numerical format (to be agreed)

