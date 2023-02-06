#!/usr/bin/env python

import serial
import time
global s

def read_robot(datastring):
	print(datastring)
	datastring=datastring.split(",")
	if len(datastring)<5:
		print "Error invalid input"
	else:
		lts,lns,heads,spds,steers=datastring[0:5]
		print(lts)
		lt=float(lts)
		ln=float(lns)
		spd=float(spds)
		steer=float(steers)
		print(lt)
		print(ln)
	
port = '/dev/ttyACM0'
s = serial.Serial(port,9600,timeout=None)


s.close()
s.open()
s.flush()
time.sleep(4)
for x in range(10):
	st=s.readline()
	read_robot(st)


#s.flush()
#s.write('$10.0,0.4:')
#print('$10.0,0.4:\r\n')
	time.sleep(0.8)
s.close()



