/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/pickanddrop.h"

PickAndDrop::PickAndDrop(supervisor_msgs::Action action) : VirtualAction(){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		container_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, container");
	}
}

bool PickAndDrop::preconditions(){
	return true;
}

bool PickAndDrop::plan(){
	return true;
}

bool PickAndDrop::exec(){
	return true;
}

bool PickAndDrop::post(){
	return true;
}
