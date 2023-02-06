import cv2
import os
import time
import sys
import datetime
import shutil




if not os.path.exists("/mnt/ramdisk/images"):
      
    # if the demo_folder directory is not present 
    # then create it.
    os.makedirs("/mnt/ramdisk/images")
else:
    shutil.rmtree("/mnt/ramdisk/images")
    os.makedirs("/mnt/ramdisk/images")
        

#cv2.namedWindow("preview")
vc = cv2.VideoCapture(1)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False
    
snap = 1

font = cv2.FONT_HERSHEY_SIMPLEX

while rval:
    #cv2.imshow("Ceres Cam", frame)
    rval, frame = vc.read()
    cv2.putText(frame, 
                str(snap), 
                (50, 50), 
                font, 1, 
                (0, 255, 255), 
                2)
    


    now = datetime.datetime.now()

    cv2.imwrite('/mnt/ramdisk/images/'+str(now)+'.jpg', frame)
    
    
    print ("saved " + str(snap))
    snap = snap + 1
    sys.stderr.write('/viewfolder/'+str(now)+'.jpg')
    time.sleep(0.5)
    if os.path.exists('/mnt/ramdisk/images/'+str(now)+'.jpg'):
        os.remove('/mnt/ramdisk/images/'+str(now)+'.jpg')

    #time.sleep(0.2)
    #sys.stderr.write("/viewfolder/dog.jpg")
    
    '''
    key = cv2.waitKey(100)
    if key == 27: # exit on ESC
        break
    '''

#cv2.destroyWindow("preview")
vc.release()
