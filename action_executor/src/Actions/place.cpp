/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/place.h"

Place::Place(supervisor_msgs::Action action) : VirtualAction(){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		support_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, support");
	}
}

bool Place::preconditions(){
	return true;
}

bool Place::plan(){
	return true;
}

bool Place::exec(){
	return true;
}

bool Place::post(){
	return true;
}
