#!/bin/bash
set -e

source /home/catkin_ws/devel/setup.bash
rospack list
python /home/catkin_ws/src/cadrl_ros/scripts/cadrl_ros_node.py

exec "$@"