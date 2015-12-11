/**
author Sandra Devin

State machine for the robot
**/

#include <state_machines/robot_sm.h>



RobotSM::RobotSM():
action_client("supervisor/action_executor", true)
 {
	node.getParam("/robot/name", robot_name);
	action_client.waitForServer();
	isActing = false;
}


/*
Called once when the goal of the action client completes
*/
void RobotSM::doneCb(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result){

		isActing = false;
}

/*
State where the robot is IDLE
*/
string RobotSM::idleState(){

	//We look if the robot has an action to do
  	ros::ServiceClient client = node.serviceClient<supervisor_msgs::GetActionTodo>("mental_state/get_action_todo");
	supervisor_msgs::GetActionTodo srv;
	srv.request.agent = robot_name;
	srv.request.actor = robot_name;
	if (client.call(srv)){
	 if(srv.response.state == "READY"){//the robot has an action to do, we send it to the action manager
		supervisor_msgs::ActionExecutorGoal goal;
  		goal.action = srv.response.action;
  		action_client.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
		isActing = true;
		ROS_INFO("[state_machines] Robot goes to ACTING");
		return "ACTING";
	 }else if(srv.response.state == "NEEDED"){//the robot has an action but not possible yet, we go to the WAITING state
		ROS_INFO("[state_machines] Robot goes to WAITING");
		return "WAITING";
	 }
	}else{
	 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_todo");
	}

	return "IDLE";
}

/*
State where the robot is ACTING
*/
string RobotSM::actingState(){

	//TODO stop order and interference
	if(!isActing){
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
	}

	return "ACTING";
}

/*
State where the robot is WAITING
*/
string RobotSM::waitingState(){

	//We look if the robot has an action to do
  	ros::ServiceClient client = node.serviceClient<supervisor_msgs::GetActionTodo>("mental_state/get_action_todo");
	supervisor_msgs::GetActionTodo srv;
	srv.request.agent = robot_name;
	srv.request.actor = robot_name;
	if (client.call(srv)){
	 if(srv.response.state == "READY"){//the robot has an action to do, we send it to the action manager
		supervisor_msgs::ActionExecutorGoal goal;
  		goal.action = srv.response.action;
  		//action_client.sendGoal(goal, &this->doneCb, &this->activeCb, &this->feedbackCb);
		isActing = true;
		ROS_INFO("[state_machines] Robot goes to ACTING");
		return "ACTING";
	 }else if(srv.response.state == "NEEDED"){//the robot is still not possible, we stay in the WAITING state
		ROS_INFO("[state_machines] Robot goes to WAITING");
		return "WAITING";
	 }else{// the robot has no more action to do, we return to IDLE
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
	  }
	}else{
	 ROS_ERROR("[state_machines] Failed to call service mental_state/get_action_todo");
	}
	
	return "WAITING";
}
