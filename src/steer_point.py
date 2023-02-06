#!/usr/bin/env python

import serial
import time
global s

def read_robot(datastring):
	print(datastring)
	datastring=datastring.split(",")
	if len(datastring)<5:
		print "Error invalid input"
		lt=0
		ln=0
		head=-1
		spd=0
		steer=0
		return (lt,ln,head,spd,steer)
	else:
		lts,lns,heads,spds,steers=datastring[0:5]
		print(lts)
		lt=float(lts)
		ln=float(lns)
		head=float(heads)
		spd=float(spds)
		steer=float(steers)
		print(lt)
		print(ln)
		return (lt,ln,head,spd,steer)
	
port = '/dev/ttyACM0'
s = serial.Serial(port,9600,timeout=None)


s.close()
s.open()
s.flush()
time.sleep(4)
for x in range(1000):
	st=s.readline()
	(lt,ln,head,spd,steer)=read_robot(st)
	if head>-1:
		st=270-head
		s.write('$%f,0.0:'%(st))
		print('Heading is:%f'%(head))
		print('Steering to:%f'%(st))

#s.flush()

#print('$10.0,0.4:\r\n')
	time.sleep(0.1)
s.close()



