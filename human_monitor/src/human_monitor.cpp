/**
author Sandra Devin

**/

#include <human_monitor/human_monitor.h>


HumanMonitor::HumanMonitor(){
	hasPicked_ = false;
}

/*
Function to call when a human picks an object
	@agent: the human who does the action
	@object: the object picked
*/
void HumanMonitor::humanPick(string agent, string object){
	
	ROS_INFO("[human_monitor] %s has picked %s", agent.c_str(), object.c_str());
	
	ros::NodeHandle node;
	ros::ServiceClient action_state = node.serviceClient<supervisor_msgs::ActionState>("mental_state/action_state");

	//put the object in the hand of the agent
	//TODO

	//we remember the action in order to recognize pickandplace
	hasPicked_ = true;

	//we create the corresponding action
	supervisor_msgs::Action action;
	action.name = "pick";
	action.parameters.push_back(object);
	action.actors.push_back(agent);

	//send the action to the mental state manager
	supervisor_msgs::ActionState srv_astate;
 	srv_astate.request.action = action;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
   	 ROS_ERROR("Failed to call service mental_state/action_state");
 	}

}

/*
Function to call when a human picks an object
	@agent: the human who does the action
	@object: the object picked
*/
void HumanMonitor::humanPlace(string agent, string object, string support){
	
	ROS_INFO("[human_monitor] %s has placed %s on %s", agent.c_str(), object.c_str(), support.c_str());
	
	ros::NodeHandle node;
	ros::ServiceClient action_state = node.serviceClient<supervisor_msgs::ActionState>("mental_state/action_state");

	//remove the object from the hand of the agent
	//TODO

	//put the object on the placement
	//TODO

	//we create the corresponding action
	supervisor_msgs::Action action;
	action.name = "place";
	action.parameters.push_back(object);
	action.parameters.push_back(support);
	action.actors.push_back(agent);

	//send the action to the mental state manager
	supervisor_msgs::ActionState srv_astate;
 	srv_astate.request.action = action;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
   	 ROS_ERROR("Failed to call service mental_state/action_state");
 	}

	if(hasPicked_){//if there was a pick, it is also a pickand place action
		hasPicked_ = false;
		//we create the corresponding action
		supervisor_msgs::Action action2;
		action2.name = "pickandplace";
		action2.parameters.push_back(object);
		action2.parameters.push_back(support);
		action2.actors.push_back(agent);

		//send the action to the mental state manager
 		srv_astate.request.action = action2;
 		srv_astate.request.state = "DONE";
  		if (!action_state.call(srv_astate)){
   		 ROS_ERROR("Failed to call service mental_state/action_state");
 		}
	}
}
