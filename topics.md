Topics
```
publat=rospy.Publisher("/CeresLat", Float64, queue_size=1)
publon=rospy.Publisher("/CeresLon", Float64, queue_size=1)
pubhead=rospy.Publisher("/CeresHead", Float64, queue_size=1)
pubAspd=rospy.Publisher("/CeresActSpd", Float64, queue_size=1)
pubAstr=rospy.Publisher("/CeresActStr", Float64, queue_size=1)
pubBatt=rospy.Publisher("/CeresBatt", Int8, queue_size=1)
pubError=rospy.Publisher("/CeresError", Int8, queue_size=1)
pubOdo=rospy.Publisher("/CeresOdo", Int16, queue_size=1)

rospy.Subscriber("/CeresSpeed", Float64, speedcallback)
rospy.Subscriber("/CeresSteer", Float64, steercallback)
rospy.Subscriber("/cmd_vel", Twist, twistcallback)
rospy.Subscriber("/CeresBuzz", Bool, buzzcallback)
rospy.Subscriber("/CeresSquirt", Int8, squirtcallback)   
```
From Arduino to ROS
```
$odos,steers,spds,lts,lns,heads,batts,errors:
```
To Arduino from ROS
```
s.write('$%2.1f,%2.1f,%d,%d:\r\n'%(steer,speed,buzz,squirt))
```
