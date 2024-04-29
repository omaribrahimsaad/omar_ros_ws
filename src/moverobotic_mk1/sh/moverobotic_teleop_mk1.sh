#!/bin/bash
if [ -f ~/catkin_ws/devel/setup.bash ]; then
    source ~/catkin_ws/devel/setup.bash
fi
if pidof rosout >/dev/null
then
	killall -9 rosmaster
	killall -9 roscore
fi
roslaunch moverobotic_mk1 moverobotic_hardware.launch
