/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/place.h"

Place::Place(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		support_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, support");
	}
}

bool Place::preconditions(){

   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
      return false;
      ROS_WARN("[action_executor] The object to place is not a known manipulable object");
   }
   
   //Then we check if the support is a known support object
   if(!isSupportObject(support_)){
      return false;
      ROS_WARN("[action_executor] The support is not a known support object");
   }
   
   //Then we check if the robot has the object in hand and if the support is reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
	fact.property = "isHoldBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
   fact.subjectId = support_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
	
	return ArePreconditionsChecked(precsTocheck);
   
	return true;
}

bool Place::plan(){
	return true;
}

bool Place::exec(Server* action_server){
	return true;
}

bool Place::post(){

   //PutInSupport(object_, support_);

	return true;
}
