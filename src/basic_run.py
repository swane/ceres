#!/usr/bin/env python

import serial
import time
global s

port = '/dev/ttyACM0'
s = serial.Serial(port,9600,timeout=None)

steer=10
speed=0.5
s.close()
s.open()
s.flush()
time.sleep(20)
for x in range(10):
    s.write('$%2.1f,%2.1f:\r\n'%(steer,speed))  #Send data to the Arduino
    print('$%2.1f,%2.1f:\r\n'%(steer,speed))
#s.flush()
#s.write('$10.0,0.4:')
#print('$10.0,0.4:\r\n')
    time.sleep(0.8)
s.close()



