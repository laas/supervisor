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
bool humaActionSimu(supervisor_msgs::HumanActionSimu::Request  &req, supervisor_msgs::HumanActionSimu::Response &res){
	
	if(req.actionName == "pick"){
	   hm->humanPick(req.agent, req.object);
	}else if(req.actionName == "place"){
	   hm->humanPlace(req.agent, req.object, req.support);
	}else{
	   ROS_ERROR("[human_monitor] Unknown action name");
	}

	return true;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_monitor");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  ROS_INFO("[human_monitor] Init human_monitor");

  //Services declarations
  ros::ServiceServer service_action = node.advertiseService("human_monitor/human_action_simu", humaActionSimu); //allows the simulation to tell that a human has done an action

  ROS_INFO("[human_monitor] human_monitor ready");

  while (node.ok()) {

  ros::spinOnce();
  loop_rate.sleep();
  }

  return 0;
}
