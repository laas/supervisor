/**
author Sandra Devin

Class allowing the execution of a pick and place action

**/

#include "action_executor/Actions/pickandplace.h"

PickAndPlace::PickAndPlace(supervisor_msgs::Action action) : VirtualAction(){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		support_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, support");
	}
}

bool PickAndPlace::preconditions(){
	return true;
}

bool PickAndPlace::plan(){
	return true;
}

bool PickAndPlace::exec(){
	return true;
}

bool PickAndPlace::post(){
	return true;
}
