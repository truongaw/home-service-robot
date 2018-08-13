#!/bin/sh
catkin_dir=/home/workspace/catkin_ws
catkin_src_dir=$catkin_dir/src

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/World/u.world" &
sleep 3

xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=$catkin_src_dir/World/gmapping_demo.launch.xml " &
sleep 3

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3

xterm -e " rosrun wall_follower wall_follower"
