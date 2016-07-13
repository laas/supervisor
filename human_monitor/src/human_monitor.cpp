/**
author Sandra Devin

**/

#include <human_monitor/human_monitor.h>

/*
Function to call when a human picks an object
	@agent: the human who does the action
	@object: the object picked
*/
void HumanMonitor::humanPick(string agent, string object){

    pair<bool, string> previousAttachment = hasInHand(agent);
    if(previousAttachment.first){
        ROS_WARN("[human_monitor] %s has already %s in hand", agent.c_str(), previousAttachment.second.c_str());
        return;
    }
	
	ROS_INFO("[human_monitor] %s has picked %s", agent.c_str(), object.c_str());
	
	ros::NodeHandle node;
    ros::ServiceClient action_state = node.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
    ros::ServiceClient state_machine = node.serviceClient<supervisor_msgs::HumanAction>("state_machines/human_action");
	ros::ServiceClient put_in_hand = node.serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");

	//put the object in the hand of the agent
	string humanHand;
	node.getParam("/humanRightHand", humanHand);
	toaster_msgs::PutInHand srv_putInHand;
    srv_putInHand.request.objectId = object;
    srv_putInHand.request.agentId = agent;
    srv_putInHand.request.jointName = humanHand;
    if (!put_in_hand.call(srv_putInHand)){
   	 ROS_ERROR("Failed to call service pdg/put_in_hand");
    }

    //we add the attachment
    pair<string, string> attach;
    attach.first = agent;
    attach.second = object;
    attachments.push_back(attach);
 	
	//we create the corresponding action
	supervisor_msgs::Action action;
	action.name = "pick";
	action.parameters.push_back(object);
	action.actors.push_back(agent);

    //send the action to the state machine manager
    supervisor_msgs::HumanAction srv_sm;
    srv_sm.request.action = action;
    srv_sm.request.agent = agent;
    if (!state_machine.call(srv_sm)){
     ROS_ERROR("Failed to call service state_machines/change_state");
    }

	//send the action to the mental state manager
    supervisor_msgs::ChangeState srv_astate;
    srv_astate.request.type = "action";
    srv_astate.request.action = action;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
 	}

}

/*
Function to call when a human place an object
	@agent: the human who does the action
	@object: the object placed
	@support: the support where the object is placed
*/
void HumanMonitor::humanPlace(string agent, string object, string support){


    pair<bool, string> previousAttachment = hasInHand(agent);
    if(!previousAttachment.first){
        ROS_WARN("[human_monitor] %s has no object in hand", agent.c_str());
        return;
    }else{
        if(previousAttachment.second != object){
            ROS_WARN("[human_monitor] %s has %s in hand, not %s", agent.c_str(), previousAttachment.second.c_str(), object.c_str());
            return;
        }
    }

	ROS_INFO("[human_monitor] %s has placed %s on %s", agent.c_str(), object.c_str(), support.c_str());
	
	ros::NodeHandle node;
    ros::ServiceClient action_state = node.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
	ros::ServiceClient remove_from_hand = node.serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    ros::ServiceClient set_entity_pose = node.serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    ros::ServiceClient state_machine = node.serviceClient<supervisor_msgs::HumanAction>("state_machines/human_action");

    string robotName;
    node.getParam("/robot/name", robotName);

	//remove the object from the hand of the agent
	toaster_msgs::RemoveFromHand srv_rmFromHand;
   srv_rmFromHand.request.objectId = object;
   if (!remove_from_hand.call(srv_rmFromHand)){
   	 ROS_ERROR("Failed to call service pdg/remove_from_hand");
 	}

   //we remove the corresponding attachment
   for(vector<pair<string, string> >::iterator it = attachments.begin(); it != attachments.end(); it++){
       if(it->first == agent){
           attachments.erase(it);
           break;
       }
   }

	//put the object on the placement
	double objectHeight, supportHeight;
	string objectHeightTopic = "/objectsHeight/bottom/";
	objectHeightTopic = objectHeightTopic + object;
	string supportHeightTopic = "/objectsHeight/top/";
	supportHeightTopic = supportHeightTopic + support;
	node.getParam(objectHeightTopic, objectHeight);
	node.getParam(supportHeightTopic, supportHeight);
    toaster_msgs::ObjectListStamped objectList;
	double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == support){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
       z = z + objectHeight + supportHeight;
       toaster_msgs::SetEntityPose srv_setPose;
       srv_setPose.request.id = object;
       srv_setPose.request.type = "object";
       srv_setPose.request.pose.position.x = x;
       srv_setPose.request.pose.position.y = y;
       srv_setPose.request.pose.position.z = z;
       srv_setPose.request.pose.orientation.x = 0.0;
       srv_setPose.request.pose.orientation.y = 0.0;
       srv_setPose.request.pose.orientation.z = 0.0;
       srv_setPose.request.pose.orientation.w = 1.0;
       if (!set_entity_pose.call(srv_setPose)){
         ROS_ERROR("Failed to call service pdg/set_entity_pose");
    	 }
   }
   catch(const std::exception & e){
       ROS_WARN("Failed to read %s pose from toaster", support.c_str());
   }

    string replacementTopic = "/humanReplacementSupport/";
    replacementTopic = replacementTopic + support;
    string replacementSupport;
    if(node.hasParam(replacementTopic)){
        node.getParam(replacementTopic, replacementSupport);
    }else{
        replacementSupport = support;
    }
	

    //send the action to the mental state manager
	supervisor_msgs::Action action;
	action.name = "place";
	action.parameters.push_back(object);
    action.parameters.push_back(replacementSupport);
    action.actors.push_back(agent);
    supervisor_msgs::ChangeState srv_astate;
    srv_astate.request.type = "action";
 	srv_astate.request.action = action;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
 	}

    //send the action to the state machine manager
    supervisor_msgs::Action action1;
    action1.name = "place";
    action1.parameters.push_back(object);
    action1.parameters.push_back(support);
    action1.actors.push_back(agent);

    supervisor_msgs::HumanAction srv_sm;
    srv_sm.request.action = action1;
    srv_sm.request.agent = agent;
    if (!state_machine.call(srv_sm)){
     ROS_ERROR("Failed to call service state_machines/change_state");
    }

	//we also consider a pick and place action
	supervisor_msgs::Action action2;
	action2.name = "pickandplace";
	action2.parameters.push_back(object);
    action2.parameters.push_back(replacementSupport);
	action2.actors.push_back(agent);

    //send the action to the mental state manager
    srv_astate.request.action = action2;
    srv_astate.request.state = "DONE";
    if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
    }

    //we also consider a pick and place reachable action
    supervisor_msgs::Action action3;
    action3.name = "pickandplacereachable";
    action3.parameters.push_back(object);
    action3.parameters.push_back(replacementSupport);
    action3.parameters.push_back(robotName);
    action3.actors.push_back(agent);

    //send the action to the mental state manager
    srv_astate.request.action = action3;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
 	}

    //we also consider a place reachable action
    supervisor_msgs::Action action4;
    action4.name = "placereachable";
    action4.parameters.push_back(object);
    action4.parameters.push_back(replacementSupport);
    action4.parameters.push_back(robotName);
    action4.actors.push_back(agent);

    //send the action to the mental state manager
    srv_astate.request.action = action4;
    srv_astate.request.state = "DONE";
    if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
    }
}

/*
Function to call when a human drop an object
	@agent: the human who does the action
	@object: the object droped
	@support: the container where the object is droped
*/
void HumanMonitor::humanDrop(string agent, string object, string container){
	
    pair<bool, string> previousAttachment = hasInHand(agent);
    if(!previousAttachment.first){
        ROS_WARN("[human_monitor] %s has no object in hand", agent.c_str());
        return;
    }else{
        if(previousAttachment.second != object){
            ROS_WARN("[human_monitor] %s has %s in hand, not %s", agent.c_str(), previousAttachment.second.c_str(), object.c_str());
            return;
        }
    }

	ROS_INFO("[human_monitor] %s has droped %s in %s", agent.c_str(), object.c_str(), container.c_str());
	
	ros::NodeHandle node;
    ros::ServiceClient action_state = node.serviceClient<supervisor_msgs::ChangeState>("mental_state/change_state");
	ros::ServiceClient remove_from_hand = node.serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    ros::ServiceClient set_entity_pose = node.serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    ros::ServiceClient state_machine = node.serviceClient<supervisor_msgs::HumanAction>("state_machines/human_action");

	//remove the object from the hand of the agent
	toaster_msgs::RemoveFromHand srv_rmFromHand;
   srv_rmFromHand.request.objectId = object;
   if (!remove_from_hand.call(srv_rmFromHand)){
   	 ROS_ERROR("Failed to call service pdg/remove_from_hand");
 	}

   //we remove the corresponding attachment
   for(vector<pair<string, string> >::iterator it = attachments.begin(); it != attachments.end(); it++){
       if(it->first == agent){
           attachments.erase(it);
           break;
       }
   }

	//put the object in the container
    toaster_msgs::ObjectListStamped objectList;
	double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == container){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
       toaster_msgs::SetEntityPose srv_setPose;
       srv_setPose.request.id = object;
       srv_setPose.request.type = "object";
       srv_setPose.request.pose.position.x = x;
       srv_setPose.request.pose.position.y = y;
       srv_setPose.request.pose.position.z = z;
       srv_setPose.request.pose.orientation.x = 0.0;
       srv_setPose.request.pose.orientation.y = 0.0;
       srv_setPose.request.pose.orientation.z = 0.0;
       srv_setPose.request.pose.orientation.w = 1.0;
       if (!set_entity_pose.call(srv_setPose)){
         ROS_ERROR("Failed to call service pdg/set_entity_pose");
         }
   }
   catch(const std::exception & e){
       ROS_WARN("Failed to read %s pose from toaster", container.c_str());
   }

	//we create the corresponding action
	supervisor_msgs::Action action;
	action.name = "drop";
	action.parameters.push_back(object);
	action.parameters.push_back(container);
	action.actors.push_back(agent);

	//send the action to the mental state manager
    supervisor_msgs::ChangeState srv_astate;
    srv_astate.request.type = "action";
 	srv_astate.request.action = action;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
 	}

	//we also consider a pick and drop action
	supervisor_msgs::Action action2;
	action2.name = "pickanddrop";
	action2.parameters.push_back(object);
	action2.parameters.push_back(container);
	action2.actors.push_back(agent);
	
    supervisor_msgs::HumanAction srv_sm;
    srv_sm.request.action = action;
    srv_sm.request.agent = agent;
    if (!state_machine.call(srv_sm)){
     ROS_ERROR("Failed to call service state_machines/change_state");
    }
    //send the action to the mental state manager
 	srv_astate.request.action = action2;
 	srv_astate.request.state = "DONE";
  	if (!action_state.call(srv_astate)){
     ROS_ERROR("Failed to call service mental_state/change_state");
 	}
}

/*
Function which return true if an agent has already an object in hand
*/
pair<bool, string> HumanMonitor::hasInHand(string agent){
    pair<bool, string> answer;
    for(vector<pair<string, string> >::iterator it = attachments.begin(); it != attachments.end(); it++){
        if(it->first == agent){
            answer.first = true;
            answer.second = it->second;
            return answer;
        }
    }
    answer.first = false;
    return answer;
}

/*
Function which return true if an object is a manipulable object (based on parameters)
    @object: the tested object
*/
bool HumanMonitor::isManipulableObject(string object){

   //first we get the manipulable objects from parameters
   vector<string> manipulableObjects;
   ros::NodeHandle node;
   node.getParam("/entities/objects", manipulableObjects);

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
bool HumanMonitor::isSupportObject(string support){

   //first we get the support objects from parameters
   vector<string> supportObjects;
   ros::NodeHandle node;
   node.getParam("/entities/supports", supportObjects);

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
bool HumanMonitor::isContainerObject(string container){

   //first we get the container objects from parameters
   vector<string> containerObjects;
   ros::NodeHandle node;
   node.getParam("/entities/containers", containerObjects);

   //Then we check if the object is in the list
   for(vector<string>::iterator it = containerObjects.begin(); it != containerObjects.end(); it++){
      if(*it == container){
         return true;
      }
   }

   return false;

}
