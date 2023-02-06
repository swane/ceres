#!/usr/bin/env python
#Simple hierarchy control. Output is muxop and muxstate
#input is topics muxiplow and muxiphigh
#the topic muxiphigh has a higher priority and will superceed muxiplow and muxop=muxiphigh. If it stops, muxsop=muxiplow
#topic muxstate=0, 1 2 depending on the input 0=none, 1 = muxiplow only , 2 = muxiphigh (superceeds)
#timeout is time in secs the state reverts to lower

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

#rostopic pub -r 10 /cmd_vel_mux/cmd_vel_nav geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0,z: 0}}'
# rostopic pub -r 10 cmd_vel_mux/cmd_vel_man geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0,z: 0.2}}'
state=0
timeout=2

lowtime=0
hightime=0




def read_twist():
    # create a twist message, fill in the details
    twist = Twist()
    twist.linear.x = x_speed;                   # our forward speed
    twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
    twist.angular.x = 0; twist.angular.y = 0;   #          or these!
    twist.angular.z = z_ang;  
    # rotation in rad / sec
    # announce move, and publish the message
    rospy.loginfo("About to be moving forward!")
    for i in range(300):
        p.publish(twist)
	rospy.loginfo("Sending Twist: [%f,%f]"%(twist.angular.z ,twist.linear.x))
        rospy.sleep(0.09) 

    # create a new message
    twist = Twist()

    # note: everything defaults to 0 in twist, if we don't fill it in, we stop!
    rospy.loginfo("Stopping!")
    p.publish(twist)
    
def cmdnavcallback(msg):
    global lowtime
    #rospy.loginfo("Low callback")
    lowtime=rospy.Time.now().to_sec()
    if state==1:
        cmdvel.publish(msg)
	
def cmdmancallback(msg):
    global hightime
    #rospy.loginfo("High callback")
    hightime=rospy.Time.now().to_sec()
    if state==2:
        cmdvel.publish(msg)
    
    
   


# this quick check means that the following code runs ONLY if this is the 
# main file -- if we "import move" in another file, this code will not execute.
if __name__=="__main__":

    # first thing, init a node!
    rospy.init_node('cmd_vel_mux',anonymous=False)

    # publish to cmd_vel
    cmdvel = rospy.Publisher('cmd_vel_mux/cmd_vel_op', Twist, queue_size=1)
    cmdstate=rospy.Publisher("cmd_vel_mux/state", Int16, queue_size=1)
    #Autonomous navigation topic
    rospy.Subscriber("cmd_vel_mux/cmd_vel_nav", Twist, cmdnavcallback)
    #Manual control topic
    rospy.Subscriber("cmd_vel_mux/cmd_vel_man", Twist, cmdmancallback)
    
    begin=rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
	s=rospy.Time.now().to_sec()
	timesincelow=s-lowtime
	timesincehigh=s-hightime
	oldstate=state
	#move up priority
	if ((timesincelow<timeout) and (state<1)):
	    state=1
	if ((timesincehigh<timeout) and (state<2)):
	    state=2
	    
	#move down priority    
	if ((timesincelow>timeout) and (state==1)):
	    state=0    
	if ((timesincehigh>timeout) and (state==2)):
	    state=1
	    
	#Send empty Twist (zero velocity) is state has just transistioned to 0
	if state==0 and oldstate>0:
	    twist = Twist()
	    cmdvel.publish(twist)
        #print begin-s
	if (s-begin)>1:  
	    #one second timer to show state value
	    #print state
            begin=s  



