/**
author Sandra Devin

Class allowing the execution of a pick action

**/

#include "action_executor/Actions/pick.h"

Pick::Pick(supervisor_msgs::Action action) : VirtualAction(){
	if(action.parameters.size() != 0){
		object = action.parameters[0];
	}else{
		ROS_WARN("Missing parameter: object to pick");
	}
}

bool Pick::preconditions(){
	return true;
}

bool Pick::plan(){
	return true;
}

bool Pick::exec(){
	return true;
}

bool Pick::post(){
	return true;
}
