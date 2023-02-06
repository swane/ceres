# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 10:44:03 2022

@author: 00751570


Convert Google MyMaps kml file into points or arrays of points for Sam's robots to follow


Updated to include lineStrings 12/09/2022
Updated to work in Ceres, taking in a KML filename as an argument and returning a json with lat, lon and 'action'
13/10/2022
*** only decodes points, not linestrings etc.. ***

"""
import sys
import json

import xml.etree.ElementTree as ET

try:
    file = sys.argv[1]
    print(file)
except:
    print('no file name')
    file = 'route.kml' # default

tree = ET.parse(file) # change to suit file movement
root = tree.getroot()

mission = []

for placecemark in root.iter('{http://www.opengis.net/kml/2.2}Placemark'):
    placemark_name = placecemark[0].text

    #print()
    #print(placemark_name)
    movement = placecemark[2].tag
    #print(movement)
    
    if 'Point' in movement:
        #print('Point: ')
        #print(placecemark[2].tag)
        #print(placecemark[2][0].text)
        point = (placecemark[2][0].text).strip().replace(' ', '').replace(',0', '')
        #print(point)
        point = point.split(',')
        point.append(placemark_name)
        #print(point)
        
        mission.append(point)

    if 'LineString' in movement:
        #print('Line string: ')
        #print(placecemark[2].tag)
        coords_array = (placecemark[2][1].text).strip().replace(' ', '').replace(',0', '')
        coord_list = list(filter(bool, coords_array.splitlines()))
        
        coarr = []

        for item in coord_list:
            item = item.split(',')
            coarr.append(item)
            #print(item)
        
        #print(coarr)
        
    if 'Polygon' in movement:
        #print('Polygon: ')
        #print(placecemark[2].tag)
        coords_array = (placecemark[2][0][0][1].text).strip().replace(' ', '').replace(',0', '')
        coord_list = list(filter(bool, coords_array.splitlines()))
        
        coarr = []

        for item in coord_list:
            item = item.split(',')
            coarr.append(item)
            #print(item)
        
        #print(coarr)
    
# to do: put text into numerical format (to be agreed)
#print('Mission')
print(mission)
mission = json.dumps(mission)
sys.stderr.write(str(mission))
