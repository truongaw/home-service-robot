#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pick;
  move_base_msgs::MoveBaseGoal drop;

  // set up the frame parameters
  pick.target_pose.header.frame_id = "map";
  pick.target_pose.header.stamp = ros::Time::now();
  drop.target_pose.header.frame_id = "map";
  drop.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  pick.target_pose.pose.position.x = 0.5;
  pick.target_pose.pose.position.y = -4.0;
  pick.target_pose.pose.orientation.w = 1.0;
  drop.target_pose.pose.position.x = 0.5;
  drop.target_pose.pose.position.y = -5.5;
  drop.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pick);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot picked up");
	sleep(5);
  }
  else
    ROS_INFO("Robot failed to pick up");

  
  
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(drop);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot dropped off");
	sleep(5);
  }
  else
    ROS_INFO("Robot failed to drop off");

  return 0;
}
