#!/bin/sh
catkin_dir=/home/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/World/u.world" &
sleep 3
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src_dir/World/myMap.yaml" &
sleep 3
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e " rosrun pick_objects pick_objects"&
sleep 3
xterm -e " rosrun add_markers add_markers"
