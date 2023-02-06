#!/usr/bin/env python3

# Matt Butler. Harper Adams University 2022
# Utility to capture images from a webcam and display in node-red on demand

import cv2
import os
import time
import sys
import datetime
import shutil

if not os.path.exists("/mnt/ramdisk/images"):
    os.makedirs("/mnt/ramdisk/images")
else:
    shutil.rmtree("/mnt/ramdisk/images")
    os.makedirs("/mnt/ramdisk/images")
        
vc = cv2.VideoCapture(0)
vc.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # doesn't work on python 2
vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False
    
snap = 1

font = cv2.FONT_HERSHEY_SIMPLEX

while rval:
    now = datetime.datetime.now()
    rval, frame = vc.read()
            
    if (snap%50 == 0) or (snap == 1):
        cv2.putText(frame, 
                    str(snap), 
                    (50, 50), 
                    font, 1, 
                    (0, 255, 255), 
                    2)
        cv2.imwrite('/mnt/ramdisk/images/'+str(now)+'.jpg', frame)
        #sys.stderr.write('/viewfolder/'+str(now)+'.jpg')
        print('/viewfolder/'+str(now)+'.jpg')
        #print ("saved " + str(snap))
        #time.sleep(0.5)
        if os.path.exists('/mnt/ramdisk/images/'+str(now)+'.jpg'):
            #os.remove('/mnt/ramdisk/images/'+str(now)+'.jpg')
            pass
       
    snap = snap + 1   

vc.release()
