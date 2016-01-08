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
	}else if(req.actionName == "drop"){
	   hm->humanDrop(req.agent, req.object, req.container);
	}else{
	   ROS_ERROR("[human_monitor] Unknown action name");
	}

	return true;
}


/*
Call back for the topic agent_monitor/fact_list
*/
void agentFactListCallback(const toaster_msgs::FactList::ConstPtr& msg){
	
	//TODO: action recognition based on distances
	
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "human_monitor");
  ros::NodeHandle node;
  ros::Rate loop_rate(30);

  ROS_INFO("[human_monitor] Init human_monitor");

  //Services declarations
  ros::ServiceServer service_action = node.advertiseService("human_monitor/human_action_simu", humaActionSimu); //allows the simulation to tell that a human has done an action
  
  ros::Subscriber sub = node.subscribe("agent_monitor/factList", 1000, agentFactListCallback);

  ROS_INFO("[human_monitor] human_monitor ready");

  ros::spin();

  return 0;
}
