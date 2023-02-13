#!/usr/bin/env python

import math

def carrot_point(CN,CE,N1,E1,N2,E2,Aim):

	
	AF=((N1-CN)*(N1-N2)+(E1-CE)*(E1-E2))/(math.sqrt((N1-N2)**2+(E1-E2)**2))

	AB=math.sqrt((N1-N2)**2+(E1-E2)**2)
	FN=N1+(N2-N1)*(AF/AB)
	FE=E1+(E2-E1)*(AF/AB)

	carrot_N=FN+(N2-N1)*(Aim/AB)

	carrot_E=FE+(E2-E1)*(Aim/AB)
	
	return carrot_N,carrot_E

carrot_N,carrot_E=carrot_point(2,6,0,0,10,12,4)
print "Carrot N", carrot_N
print "Carrot E", carrot_E
