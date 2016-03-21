/**
author Sandra Devin

Virtual class to represent an action, contain general function for the execution of actions

**/

#include <action_executor/virtual_action.h>

VirtualAction::VirtualAction(Connector* connector){

   node_.getParam("/simu", simu_);
   node_.getParam("/robot/name", robotName_);
   node_.getParam("/waitActionServer", waitActionServer_);
   node_.getParam("/nbPlanMaxGTP", nbPlanMax_);
   connector_ = connector;
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
	   ROS_ERROR("[action_executor] Failed to call service mental_states/facts_are_in");
	}
   return false;
}

/*
Function which puts an object in the hand of the robot
	@object: the object to put
	@hand: the hand to attach to (right or left)
*/
void VirtualAction::PutInHand(string object, string hand){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");

	//put the object in the hand of the agent
	string robotHand;
	string handTopic = "/robot/hands/";
	handTopic = handTopic + hand;
	node_.getParam(handTopic, robotHand);
	toaster_msgs::PutInHand srv;
   srv.request.objectId = object;
   srv.request.agentId = robotName_;
   srv.request.jointName = robotHand;
   if (!client.call(srv)){
   	 ROS_ERROR("[action_executor] Failed to call service pdg/put_in_hand");
 	}
   
}

/*
Function which remove an object from the hand of the robot
	@object: the object to remove
*/
void VirtualAction::RemoveFromHand(string object){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");

	//remove the object from the hand of the agent
	toaster_msgs::RemoveFromHand srv;
   srv.request.objectId = object;
   if (!client.call(srv)){
   	 ROS_ERROR("[action_executor] Failed to call service pdg/remove_from_hand");
 	}
   
}


/*
Function which puts an object in the hand of the robot
	@object: the object to put
	@hand: the hand to attach to (right or left)
*/
void VirtualAction::PutInSupport(string object, string support){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");

   double objectHeight, supportHeight;
	string objectHeightTopic = "/objectsHeight/bottom/";
	objectHeightTopic = objectHeightTopic + object;
	string supportHeightTopic = "/objectsHeight/top/";
	supportHeightTopic = supportHeightTopic + support;
	node_.getParam(objectHeightTopic, objectHeight);
	node_.getParam(supportHeightTopic, supportHeight);
	toaster_msgs::ObjectList objectList;
	double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectList>("pdg/objectList",ros::Duration(1)));
       for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == support){
            x = it->meEntity.positionX;
            y = it->meEntity.positionY;
            z = it->meEntity.positionZ;
            break;
         }
       }
       z = z + objectHeight + supportHeight;
       toaster_msgs::SetEntityPose srv;
       srv.request.id = object;
       srv.request.type = "object";
       srv.request.x = x;
       srv.request.y = y;
       srv.request.z = z;
       srv.request.roll = 0.0;
       srv.request.pitch = 0.0;
       srv.request.yaw = 0.0;
       if (!client.call(srv)){
      	 ROS_ERROR("Failed to call service toaster_simu/set_entity_pose");
    	 }
   }
   catch(const std::exception & e){
       ROS_WARN("[action_executor] Failed to read %s pose from toaster", support.c_str());
   }
   
}

/*
Function which update GTP world state with TOASTER
*/
bool VirtualAction::updateGTP(){
   
  // send goal to GTP
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "update";
  connector_->acGTP_->sendGoal(goal);

  //wait for the action to return
  bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(waitActionServer_));

  if (!finishedBeforeTimeout){
    ROS_INFO("[action_executor] GTP Action did not finish before the time out.");
    return false;
  }
   
   return true;
}

/*
Function which ask a plan to GTP and return the id of the solution (-1 if no solution)
*/
int VirtualAction::planGTP(string actionName, vector<gtp_ros_msg::Ag> agents, vector<gtp_ros_msg::Obj> objects, vector<gtp_ros_msg::Data> datas, vector<gtp_ros_msg::Points> points){
  
  updateGTP();
  
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "planning";
  goal.req.actionName = actionName;
  goal.req.involvedAgents = agents;
  goal.req.involvedObjects = objects;
  goal.req.data = datas;
  goal.req.points = points;
  goal.req.predecessorId.actionId = connector_->previousId_;
  goal.req.predecessorId.alternativeId = 0;
  
  int nbTry = 0;
  while(nbTry < nbPlanMax_){
     connector_->acGTP_->sendGoal(goal);
     bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(waitActionServer_));

     if (finishedBeforeTimeout)
     {
       if(connector_->acGTP_->getResult()->ans.success){
           return connector_->acGTP_->getResult()->ans.identifier.actionId;
       }
     }
     else{
         ROS_INFO("GTP Action did not finish before the time out.");
         return -1;
     }
       
      nbTry++;
   }
   
   return -1;
}


