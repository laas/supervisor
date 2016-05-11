/**
author Sandra Devin

Main class of the goal_manager.

The plan elaboration allows to find a plan to execute a goal

**/

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

using namespace std;


int main (int argc, char **argv)
{
  ros::init(argc, argv, "plan_elaboration");
  ros::NodeHandle node;

  ROS_INFO("[goal_manager] Init plan_elaboration");
 
  ros::spin();

  return 0;
}
