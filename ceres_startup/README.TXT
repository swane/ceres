add this into ardusimple.launch at the end:


<include file="$(find yocs_cmd_vel_mux)/launch/standalone.launch" />



or launch manually:

roslaunch yocs_cmd_vel_mux standalone.launch