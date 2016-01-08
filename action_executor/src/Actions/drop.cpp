/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/drop.h"

Drop::Drop(supervisor_msgs::Action action) : VirtualAction(){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		container_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, container");
	}
}

bool Drop::preconditions(){
	return true;
}

bool Drop::plan(){
	return true;
}

bool Drop::exec(){
	return true;
}

bool Drop::post(){
	return true;
}
