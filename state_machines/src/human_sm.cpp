/**
author Sandra Devin

State machine for the robot
**/

#include <state_machines/human_sm.h>



HumanSM::HumanSM(string _human_name):
action_client("supervisor/action_executor", true)
 {
	human_name = _human_name;
	node.getParam("/robot/name", robot_name);
	action_client.waitForServer();
}

/*
State where the human is IDLE
*/
string HumanSM::idleState(){

	//TODO if human ACTING
	//TODO if human PRESENT

	//We look if the human thinks he has an action to do
  	ros::ServiceClient client = node.serviceClient<supervisor_msgs::GetActionTodo>("mental_state/get_action_todo");
  	ros::ServiceClient client_state = node.serviceClient<supervisor_msgs::GetActionState>("mental_state/get_action_state");
	supervisor_msgs::GetActionTodo srv_todo;
	supervisor_msgs::GetActionState srv_state;
	srv_todo.request.agent = human_name;
	srv_todo.request.actor = human_name;
	if (client.call(srv_todo)){
	 if(srv_todo.response.state == "READY"){//the human thinks he has an action to do
		//we look if the robot also thinks the human should do the action
		srv_state.request.agent = robot_name;
		srv_state.request.action = srv_todo.response.action;
		if (client.call(srv_state)){
		 if(srv_state.response.state == "READY"){//the state is the same in the robot knowledge, the human SOULD ACT
			ROS_INFO("[state_machines] %s goes to SHOULD ACT", human_name.c_str());
			return "SHOULD_ACT";
		 }else{//it is necessary to solve the divergent belief
			//TODO Divergent belief
		  }
		}else{
	 	 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_state");
		}
	 }else if(srv_todo.response.state == "NEEDED"){//the human thinks he has an actio nto do but no possible
		//we look if the robot also thinks the human should do the action and that the action is not possible
		srv_state.request.agent = robot_name;
		srv_state.request.action = srv_todo.response.action;
		if (client.call(srv_state)){
		 if(srv_state.response.state == "NEEDED"){//the state is the same in the robot knowledge, the human has to WAIT
			ROS_INFO("[state_machines] %s goes to WAITING", human_name.c_str());
			return "WAITING";
		 }else{//it is necessary to solve the divergent belief
			//TODO Divergent belief
		  }
		}else{
	 	 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_state");
		}
	 }else{//the human thinks he has no action, we check if the robot thinks he has an action to do	
		srv_todo.request.agent = robot_name;
		if (client.call(srv_todo)){
		  if(srv_todo.response.state == "READY"){//the robot thinks the human should act, it is necessary to solve the divergent belief
			//TODO Divergent belief
		  }
		}else{
		 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_todo");
		}
	  }
	}else{
	 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_todo");
	}

	return "IDLE";
}

/*
State where the human is ACTING
*/
string HumanSM::actingState(){

	//TODO sill ACTING?
	//TODO if human PRESENT
	

	return "ACTING";
}

/*
State where the human is WAITING
*/
string HumanSM::absentState(){

	//TODO if human ACTING
	//TODO if human PRESENT
	
	
	return "ABSENT";
}

/*
State where the human is WAITING
*/
string HumanSM::waitingState(){

	//TODO if human ACTING
	//TODO if human PRESENT
	
	
	return "WAITING";
}

/*
State where the human SHOULD ACT
*/
string HumanSM::shouldActState(){

	
	//TODO if human ACTING
	//TODO if human PRESENT
	
	return "SHOULDACT";
}
