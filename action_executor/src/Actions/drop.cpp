/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/drop.h"

Drop::Drop(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		container_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, container");
	}
}

bool Drop::preconditions(){

   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
      return false;
      ROS_WARN("[action_executor] The object to place is not a known manipulable object");
   }
   
   //First we check if the container is a known container object
   if(!isContainerObject(container_)){
      return false;
      ROS_WARN("[action_executor] The container is not a known container object");
   }
   
   //Then we check if the robot has the object in hand and if the object is reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
	fact.property = "isHoldBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
   fact.subjectId = container_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
	
	return ArePreconditionsChecked(precsTocheck);
}

bool Drop::plan(){
	return true;
}

bool Drop::exec(Server* action_server){
	return true;
}

bool Drop::post(){
	return true;
}
