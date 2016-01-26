/**
author Sandra Devin

Virtual class to represent an action, contain general function for the execution of actions

**/

#include <action_executor/virtual_action.h>

VirtualAction::VirtualAction(){

   node_.getParam("/robot/name", robotName_);

}

/*
Function which return true if an object is a manipulable object (based on parameters)
	@object: the tested object
*/
bool VirtualAction::isManipulableObject(string object){

   //first we get the manipulable object from parameters
   vector<string> manipulableObjects;
   node_.getParam("/entities/objects", manipulableObjects);
   
   //Then we check if the object is in the list
   for(vector<string>::iterator it = manipulableObjects.begin(); it != manipulableObjects.end(); it++){
      if(*it == object){
         return true;
      }
   }
   
   return false;
   
}

/*
Function which return true if an object is a support object (based on parameters)
	@support: the tested object
*/
bool VirtualAction::isSupportObject(string support){

   //first we get the manipulable object from parameters
   vector<string> supportObjects;
   node_.getParam("/entities/supports", supportObjects);
   
   //Then we check if the object is in the list
   for(vector<string>::iterator it = supportObjects.begin(); it != supportObjects.end(); it++){
      if(*it == support){
         return true;
      }
   }
   
   return false;
   
}

/*
Function which return true if an object is a container object (based on parameters)
	@container: the tested object
*/
bool VirtualAction::isContainerObject(string container){

   //first we get the manipulable object from parameters
   vector<string> containerObjects;
   node_.getParam("/entities/containers", containerObjects);
   
   //Then we check if the object is in the list
   for(vector<string>::iterator it = containerObjects.begin(); it != containerObjects.end(); it++){
      if(*it == container){
         return true;
      }
   }
   
   return false;
   
}

/*
Function which check if the preconditions are in the knowledge of the robot
	@precs: list of facts to check
*/
bool VirtualAction::ArePreconditionsChecked(vector<toaster_msgs::Fact> precs){

   ros::ServiceClient client = node_.serviceClient<supervisor_msgs::FactsAreIn>("mental_state/facts_are_in");
   supervisor_msgs::FactsAreIn srv;
	srv.request.agent = robotName_;
	srv.request.facts = precs;
	if (client.call(srv)){
		return srv.response.result;
	}else{
	   ROS_ERROR("[mental_state] Failed to call service mental_states/facts_are_in");
	}
   return false;
}

