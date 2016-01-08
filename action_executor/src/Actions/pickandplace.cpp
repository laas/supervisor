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
   
   //Then we check if the support and the object are reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
   fact.subjectId = support_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
	
	return ArePreconditionsChecked(precsTocheck);
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
