/**
author Sandra Devin

Simple dialogue node

**/

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dialogue_node");
  ros::NodeHandle node;


  ROS_INFO("[dialogue_node] dialogue_node ready");

   ros::spin();

  return 0;
}
