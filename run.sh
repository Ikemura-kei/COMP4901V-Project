#! /bin/sh

# roscore &

rosparam set /custom_gesture False
roslaunch hkust_rgd_bringup onebot_bringup.launch  &

sleep 5
roslaunch hkust_rgd_behavior behavior.launch &

# sleep 2
# source ~/Dev/Voice-Module/devel/setup.bash
# roslaunch slu_model slu.launch &
