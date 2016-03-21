/**
author Sandra Devin

Main class of the action executor

This module allows the robot to execute high level actions.
Available actions:

In progess: pick, place (+pickandplace), drop (+pickanddrop)

TODO: placereachable (+pickandplacereachable), give/grab, moveTo, goTo (+engage/disengage), scan, sweep

**/

#include <action_executor/action_executor.h>



ActionExecutor::ActionExecutor(string name):
action_server_(node_, name, 
	boost::bind(&ActionExecutor::execute,this, _1), false)
 {
 	action_server_.start();
 	ROS_INFO("[action_executor] Action server ready");
}


void ActionExecutor::execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal) {
  	ros::ServiceClient client = node_.serviceClient<supervisor_msgs::ActionState>("mental_state/action_state");
	supervisor_msgs::ActionState srv;
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
		return;
	}


	//send the fact that the action is in progress to the MS Manager
	srv.request.state = "PROGRESS";
	if (!client.call(srv)){
	 ROS_ERROR("[state_machines] Failed to call service mental_state/action_state");
	}

	//Checking preconditions
	feedback_.state = "PREC";
	action_server_.publishFeedback(feedback_);
	if(!act->preconditions()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[state_machines] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
		return;
	}

	//Plan for the action
	feedback_.state = "PLAN";
	action_server_.publishFeedback(feedback_);
	if(!act->plan()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[state_machines] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
		return;
	}
	

	//Execution of the action
	feedback_.state = "EXEC";
	action_server_.publishFeedback(feedback_);
	if(!act->exec()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[state_machines] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
		return;
	}

	//Apply/Check Post-conditions
	feedback_.state = "POST";
	action_server_.publishFeedback(feedback_);
	if(!act->post()){
		srv.request.state = "FAILED";
		if (!client.call(srv)){
		 ROS_ERROR("[state_machines] Failed to call service mental_state/action_state");
		}
		result_.report = false;
		if(!action_server_.isPreemptRequested()){
			result_.state = feedback_.state;
			action_server_.setAborted(result_);
		}else{
			result_.state = "PREEMPTED";
			action_server_.setPreempted(result_);

		}
		return;
	}
	
	srv.request.state = "DONE";
	if (!client.call(srv)){
	 ROS_ERROR("[state_machines] Failed to call service mental_state/action_state");
	}
	result_.report = true;
	result_.state = "OK";
	action_server_.setSucceeded(result_);
}


VirtualAction* ActionExecutor::initializeAction(supervisor_msgs::Action action) {
	VirtualAction* act = NULL;

	if(action.name == "pick"){
		act = new Pick(action);
	}else if(action.name == "place"){
		act = new Place(action);
	}else if(action.name == "pickandplace"){
		act = new PickAndPlace(action);
	}else if(action.name == "drop"){
		act = new Drop(action);
	}else if(action.name == "pickanddrop"){
		act = new PickAndDrop(action);
	}else if(action.name == "moveTo"){
		act = new MoveTo(action);
	}else{
		ROS_WARN("[action_executor] Unknown action");
	}	

	return act;
}


void initializePr2motion() {
 
  ros::NodeHandle n;
  actionlib::SimpleActionClient<pr2motion::InitAction> init("pr2motion/Init", true);
  ros::ServiceClient connect = n.serviceClient<pr2motion::connect_port>("pr2motion/connect_port");

  ROS_INFO("Waiting for pr2motion action server to start.");
  // wait for the action server to start
  init.waitForServer(); 

  ROS_INFO("pr2motion Action server started, initialization.");
  
  //init
  pr2motion::InitGoal goal_init;
  init.sendGoal(goal_init);
  
  pr2motion::connect_port srv;
  srv.request.local = "joint_state";
  srv.request.remote = "joint_states";
  if (!connect.call(srv)){
    ROS_ERROR("[mental_state] Failed to call service pr2motion/connect_port");
  }
  srv.request.local = "head_controller_state";
  srv.request.remote = "/head_traj_controller/state";
  if (!connect.call(srv)){
    ROS_ERROR("[mental_state] Failed to call service pr2motion/connect_port");
  }
  srv.request.local = "traj";
  srv.request.remote = "gtp_trajectory";
  if (!connect.call(srv)){
    ROS_ERROR("[mental_state] Failed to call service pr2motion/connect_port");
  }
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_executor");

  ROS_INFO("[action_executor] Init action_executor");
  
  initializePr2motion();

  ActionExecutor executor("supervisor/action_executor");

  ros::spin();

  return 0;
}
