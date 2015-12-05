/**
author Sandra Devin

Main class of the state_machines manager.

The state machines manager keep trace of the activity of each agent.

**/

#include <ros/ros.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "state_machines");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  ROS_INFO("[mental_state] Init state_machines");

  //Services declarations

  ROS_INFO("[mental_state] state_machines ready");

  while (node.ok()) {

  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}
