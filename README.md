# ceres
Ceres agricultural electric quadbike <br>
publish points as Paths <br>
Path read by nav program<br>

# PID
http://wiki.ros.org/pid
```
$ roslaunch pid servo_sim.launch
```
## Dynamic reconfigure
```
rosrun rqt_reconfigure rqt_reconfigure
```
```
<node name="controller" pkg="pid" type="controller" ns="left_wheel" output="screen" >
      <param name="node_name" value="left_wheel_pid" />
      <param name="Kp" value="5.0" />
      <param name="Ki" value="0.0" />
      <param name="Kd" value="0.1" />
      <param name="upper_limit" value="10" />
      <param name="lower_limit" value="-10" />
      <param name="windup_limit" value="10" />
      <param name="max_loop_frequency" value="100.0" />
      <param name="min_loop_frequency" value="100.0" />
      <remap from="setpoint" to="/setpoint" />
     </node>
```
## Topics
- setpoint: The controller subscribes to this topic and reads std_msgs/Float64 messages. The message data element must contain the desired value of the state measurement of the controlled process.
- state: The controller subscribes to this topic and reads std_msgs/Float64 messages. The message data element must contain the current value of the controlled plant property. The controller publishes std_msgs/Float64 messages on the control_effort topic each time it receives a message on the state topic. Thus the rate at which the plant publishes state governs the control-loop rate - the plant should publish state at the desired loop rate.
- control_effort: The control_effort message data element contains the control effort to be applied to the process to drive state/data to equal setpoint/data. 
