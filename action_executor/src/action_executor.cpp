/**
author Sandra Devin

Main class of the action executor

This module allows the robot to execute high level actions.
Available actions:

In progess: pick, place (+pickandplace), drop (+pickanddrop)

TODO: placereachable (+pickandplacereachable), give/grab, moveTo, goTo (+engage/disengage), scan, sweep

**/

#include <action_executor/action_executor.h>

ros::NodeHandle* node_;
Connector* connector_;

ActionExecutor::ActionExecutor(string name):
action_server_(node_, name, 
	boost::bind(&ActionExecutor::execute,this, _1), false)
 {
 	action_server_.start();
 	ROS_INFO("[action_executor] Action server ready");
}


void ActionExecutor::execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal) {
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
    supervisor_msgs::ChangeState srv;
    srv.request.type= "action";
	srv.request.action = goal->action;

	//Getting action informations
	string name = goal->action.name;
	int id = goal->action.id;
	ROS_INFO("Executing action %s with id %i with parameters:", name.c_str(), id);
	for(int i=0; i<goal->action.parameters.size();i++){
		ROS_INFO("  %s", goal->action.parameters[i].c_str());
	}

	//Creating the action
	VirtualAction* act = NULL;
	act = initializeAction(goal->action);
	if(!act){
		ROS_WARN("Aborted");
		result_.report = false;
		result_.state = "NON_VALID";
		action_server_.setAborted(result_);
	   ROS_INFO("[action_executor] Action failed at creation");
		return;
	}

    ROS_ERROR("[action_executor] Send service");
	//send the fact that the action is in progress to the MS Manager
	srv.request.state = "PROGRESS";
	if (!client.call(srv)){
	 ROS_ERROR("[action_executor] Failed to call service mental_state/action_state");
    }
    ROS_ERROR("[action_executor] Answer service");

	//Checking preconditions
	feedback_.state = "PREC";
	action_server_.publishFeedback(feedback_);
	if(!act->preconditions()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[action_executor] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in preconditions");
		return;
	}

	//Plan for the action
	feedback_.state = "PLAN";
	action_server_.publishFeedback(feedback_);
	if(!act->plan()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[action_executor] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in planning");
		return;
	}

	//Execution of the action
	feedback_.state = "EXEC";
	action_server_.publishFeedback(feedback_);
	if(!act->exec()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[action_executor] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in execution");
		return;
	}

	//Apply/Check Post-conditions
	feedback_.state = "POST";
	action_server_.publishFeedback(feedback_);
	if(!act->post()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[action_executor] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
	   ROS_INFO("[action_executor] Action failed in post conditions");
		return;
	}
	
	srv.request.state = "DONE";
	if (!client.call(srv)){
	 ROS_ERROR("[action_executor] Failed to call service mental_state/action_state");
	}
	result_.report = true;
	result_.state = "OK";
	action_server_.setSucceeded(result_);
	ROS_INFO("[action_executor] Action succeed");
}


VirtualAction* ActionExecutor::initializeAction(supervisor_msgs::Action action) {
	VirtualAction* act = NULL;

	if(action.name == "pick"){
		act = new Pick(action, connector_);
	}else if(action.name == "place"){
		act = new Place(action, connector_);
	}else if(action.name == "pickandplace"){
		act = new PickAndPlace(action, connector_);
	}else if(action.name == "drop"){
		act = new Drop(action, connector_);
	}else if(action.name == "pickanddrop"){
		act = new PickAndDrop(action, connector_);
	}else if(action.name == "moveTo"){
		act = new MoveTo(action, connector_);
	}else{
		ROS_WARN("[action_executor] Unknown action");
	}	

	return act;
}



int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle node;
  node_ = &node;
   
  Connector connector;
  connector_ = &connector;

  ROS_INFO("[action_executor] Init action_executor");

  ActionExecutor executor("supervisor/action_executor");

  ros::spin();

  return 0;
}
