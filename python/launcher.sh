#!/bin/bash

	gnome-terminal -x bash -c "roslaunch atlas_utils vrc_task_3.launch;bash" & 
	first=$!
	gnome-terminal -x bash -c "rosrun spacenav_node spacenav_node;bash" & 
	second=$!
	gnome-terminal -x bash -c "../bin/spnav_all;bash" &
	third=$!
	gnome-terminal -x bash -c "roslaunch atlas_utils keyboard_teleop.launch" &
	fourth=$!
	gnome-terminal -x bash -c "./cyberglove.py;bash" &
	fifth=$!

#while :
#do
#	read -t 1 -n 1 key
#	
#	if [[ $key = q ]]
#	then
#		#kill -9 $first
#		#kill -9 $second
#		#kill -9 $third
#		#kill -9 $fourth
#		#kill -9 $fifth
#		kill -- -$(ps opgid= $first)
#		kill -- -$(ps opgid= $second)
#		kill -- -$(ps opgid= $third)
#		kill -- -$(ps opgid= $fourth)
#		kill -- -$(ps opgid= $fifth)
#		break
#	fi
#done

#echo $first
#echo $second
#echo $third
#echo $fourth
#echo $fifth


#pkill -TERM -P $$
