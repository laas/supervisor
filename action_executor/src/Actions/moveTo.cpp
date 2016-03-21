/**
author Sandra Devin

Class allowing the execution of a pick action

**/

#include "action_executor/Actions/moveTo.h"

MoveTo::MoveTo(supervisor_msgs::Action action) : VirtualAction(){
	if(action.parameters.size() == 1){
		position_ = action.parameters[0];
	}else{
		ROS_WARN("[action_executor] Wrong parameters number, should be: position");
	}
}

bool MoveTo::preconditions(){


   //We check if the position is valid
   string posTopic = "moveToPositions/";
   posTopic = posTopic + position_;
   if(!node_.hasParam(posTopic)){
      ROS_WARN("[action_executor] Wrong position name");
      return false;
   }
	
	return true;
 
}

bool MoveTo::plan(){
	return true;
}

bool MoveTo::exec(){
	return true;
}

bool MoveTo::post(){

	return true;
}
