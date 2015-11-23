/**
author Sandra Devin

Main class of the human_monitor.

The human monitor allows to monitor humans actions by looking into facts from toaster

**/

#include <human_monitor/human_monitor.h>

HumanMonitor* hm = new HumanMonitor();

/*
Service call from simulation to tell that a human picked an object
*/
bool humanPick(supervisor_msgs::HumanPick::Request  &req, supervisor_msgs::HumanPick::Response &res){
	
	
	hm->humanPick(req.agent, req.object);

	return true;
}

/*
Service call from simulation to tell that a human placed an object on a support
*/
bool humanPlace(supervisor_msgs::HumanPlace::Request  &req, supervisor_msgs::HumanPlace::Response &res){
	
	
	hm->humanPlace(req.agent, req.object, req.support);

	return true;
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_monitor");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  ROS_INFO("[mental_state] Init human_monitor");

  //Services declarations
  ros::ServiceServer service_pick = node.advertiseService("human_monitor/human_pick", humanPick); //allows the simulation to tell that a human picked an object
  ros::ServiceServer service_place = node.advertiseService("human_monitor/human_place", humanPlace); //allows the simulation to tell that a human place an object

  ROS_INFO("[mental_state] human_monitor ready");

  while (node.ok()) {

  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}
