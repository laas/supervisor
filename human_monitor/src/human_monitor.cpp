/**
author Sandra Devin
**/

#include <human_monitor/human_monitor.h>


/**
 * \brief Construction of the class
 * @param node pointer to the node handle
 * */
HumanMonitor::HumanMonitor(ros::NodeHandle* node){

    node_ = node;

    //Initialize the parameters
    node_->getParam("/entities/objects", manipulableObjects_);
    node_->getParam("/entities/supports", supportObjects_);
    node_->getParam("/entities/containers", containerObjects_);
    node_->getParam("/supervisor/simu", simu_);
    node_->getParam("/supervisor/robot/name", robotName_);
    node_->getParam("/human_monitor/rightHand", humanHand_);

    currentId_ = -2;

    //Initialize the publishers
    current_pub_ = node_->advertise<supervisor_msgs::ActionsList>("/human_monitor/current_humans_action", 1);
    previous_pub_ = node_->advertise<supervisor_msgs::ActionsList>("/data_manager/add_data/previous_actions", 1);

    //Initialize the clients
    client_put_hand_ = node_->serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");
    client_remove_hand_ = node_->serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    client_set_pose_ = node_->serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    client_set_info_ = node_->serviceClient<toaster_msgs::SetInfoDB>("database_manager/set_info");

    //if we are in simulation, the objects position are manager by toaster_simu, else by pdg
    if(simu_){
        client_set_pose_ = node_->serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");
    }else{
        client_set_pose_ = node_->serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    }

}

/**
 * \brief Function to call when a human picks an object
 * @param agent the human who does the action
 * @param object the object picked
 * */
void HumanMonitor::humanPick(std::string agent, std::string object){

    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
    if(previousAttachment.first){
        ROS_WARN("[human_monitor] %s has already %s in hand", agent.c_str(), previousAttachment.second.c_str());
        return;
    }

    ROS_INFO("[human_monitor] %s has picked %s", agent.c_str(), object.c_str());

    //we create the corresponding action
    supervisor_msgs::Action action;
    action.name = "pick";
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(object);
    action.actors.push_back(agent);
    action.headFocus = object;
    action.id = currentId_;
    currentId_--; //we use negative id to distinguish from robot actions

    //we publish the action in the current publisher
    supervisor_msgs::ActionsList msg_current;
    msg_current.actions.push_back(action);
    current_pub_.publish(msg_current);

    //put the object in the hand of the agent
    toaster_msgs::PutInHand srv_putInHand;
    srv_putInHand.request.objectId = object;
    srv_putInHand.request.agentId = agent;
    srv_putInHand.request.jointName = humanHand_;
    if (!client_put_hand_.call(srv_putInHand)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/put_in_hand");
    }

    //we add the attachment
    std::pair<std::string, std::string> attach;
    attach.first = agent;
    attach.second = object;
    attachments_.push_back(attach);

    //we publish the action in the previous publisher
    supervisor_msgs::ActionsList msg_previous;
    action.succeed = true;
    msg_previous.actions.push_back(action);
    previous_pub_.publish(msg_previous);
}

/**
 * \brief Function to call when a human places an object
 * @param agent the human who does the action
 * @param object the object placed
 * @param support the support where the object is placed
 * */
void HumanMonitor::humanPlace(std::string agent, std::string object, std::string support){


    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
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

    //we create the corresponding action
    supervisor_msgs::Action action;
    action.name = "place";
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(object);
    action.parameter_keys.push_back("support");
    action.parameter_values.push_back(support);
    action.actors.push_back(agent);
    action.headFocus = support;
    action.id = currentId_;
    currentId_--; //we use negative id to distinguish from robot actions

    //we publish the action in the current publisher
    supervisor_msgs::ActionsList msg_current;
    msg_current.actions.push_back(action);
    current_pub_.publish(msg_current);

    //remove the object from the hand of the agent
    toaster_msgs::RemoveFromHand srv_rmFromHand;
   srv_rmFromHand.request.objectId = object;
   if (!client_remove_hand_.call(srv_rmFromHand)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/remove_from_hand");
    }

   //we remove the corresponding attachment
   for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
       if(it->first == agent){
           attachments_.erase(it);
           break;
       }
   }

    //put the object on the placement
    double objectHeight, supportHeight;
    std::string objectHeightTopic = "/entities/objectsHeight/bottom/";
    objectHeightTopic = objectHeightTopic + object;
    std::string supportHeightTopic = "/entities/objectsHeight/top/";
    supportHeightTopic = supportHeightTopic + support;
    node_->getParam(objectHeightTopic, objectHeight);
    node_->getParam(supportHeightTopic, supportHeight);
    toaster_msgs::ObjectListStamped objectList;

    double x,y,z;
    objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
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
    if (!client_set_pose_.call(srv_setPose)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/set_entity_pose");
    }

    std::string replacementTopic = "human_monitor/humanReplacementSupport/";
    replacementTopic = replacementTopic + support;
    std::string replacementSupport;
    if(node_->hasParam(replacementTopic)){
        //we update the action
        node_->getParam(replacementTopic, replacementSupport);
        action.parameter_keys.clear();
        action.parameter_values.clear();
        action.parameter_keys.push_back("object");
        action.parameter_values.push_back(object);
        action.parameter_keys.push_back("support");
        action.parameter_values.push_back(replacementTopic);
    }

    //we publish the action in the previous publisher
    supervisor_msgs::ActionsList msg_previous;
    action.succeed = true;
    msg_previous.actions.push_back(action);
    previous_pub_.publish(msg_previous);
}

/**
 * \brief Function to call when a human drops an object
 * @param agent the human who does the action
 * @param object the object droped
 * @param container the container where the object is droped
 * */
void HumanMonitor::humanDrop(std::string agent, std::string object, std::string container){

    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
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

    //we create the corresponding action
    supervisor_msgs::Action action;
    action.name = "drop";
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(object);
    action.parameter_keys.push_back("container");
    action.parameter_values.push_back(container);
    action.actors.push_back(agent);
    action.headFocus = container;
    action.id = currentId_;
    currentId_--; //we use negative id to distinguish from robot actions

    //we publish the action in the current publisher
    supervisor_msgs::ActionsList msg_current;
    msg_current.actions.push_back(action);
    current_pub_.publish(msg_current);

    //remove the object from the hand of the agent
    toaster_msgs::RemoveFromHand srv_rmFromHand;
   srv_rmFromHand.request.objectId = object;
   if (!client_remove_hand_.call(srv_rmFromHand)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/remove_from_hand");
    }

   //we remove the corresponding attachment
   for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
       if(it->first == agent){
           attachments_.erase(it);
           break;
       }
   }

    //put the object in the container
    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
    objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
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
    if (!client_set_pose_.call(srv_setPose)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/set_entity_pose");
    }


    //we publish the action in the previous publisher
    supervisor_msgs::ActionsList msg_previous;
    action.succeed = true;
    msg_previous.actions.push_back(action);
    previous_pub_.publish(msg_previous);
}


/**
 * \brief Function to call when a human places a stick
 * @param agent the human who does the action
 * @param object the object placed
 * @param support1 the first support where the stick is placed
 * @param support2 the second support where the stick is placed
 * */
void HumanMonitor::humanPlaceStick(std::string agent, std::string object, std::string support1, std::string support2){

    std::pair<bool, std::string> previousAttachment = hasInHand(agent);
    if(!previousAttachment.first){
        ROS_WARN("[human_monitor] %s has no object in hand", agent.c_str());
        return;
    }else{
        if(previousAttachment.second != object){
            ROS_WARN("[human_monitor] %s has %s in hand, not %s", agent.c_str(), previousAttachment.second.c_str(), object.c_str());
            return;
        }
    }

    ROS_INFO("[human_monitor] %s has placed %s on %s and %s", agent.c_str(), object.c_str(), support1.c_str(), support2.c_str());

    //we create the corresponding action
    supervisor_msgs::Action action;
    action.name = "placeStick";
    action.parameter_keys.push_back("object");
    action.parameter_values.push_back(object);
    action.parameter_keys.push_back("support1");
    action.parameter_values.push_back(support1);
    action.parameter_keys.push_back("support2");
    action.parameter_values.push_back(support2);
    action.actors.push_back(agent);
    action.headFocus = support1;
    action.id = currentId_;
    currentId_--; //we use negative id to distinguish from robot actions

    //we publish the action in the current publisher
    supervisor_msgs::ActionsList msg_current;
    msg_current.actions.push_back(action);
    current_pub_.publish(msg_current);

    //remove the object from the hand of the agent
    toaster_msgs::RemoveFromHand srv_rmFromHand;
   srv_rmFromHand.request.objectId = object;
   if (!client_remove_hand_.call(srv_rmFromHand)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/remove_from_hand");
    }

   //we remove the corresponding attachment
   for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
       if(it->first == agent){
           attachments_.erase(it);
           break;
       }
   }

    //put the object on the placement
    double objectHeight, supportHeight;
    std::string objectHeightTopic = "/entities/objectsHeight/bottom/";
    objectHeightTopic = objectHeightTopic + object;
    std::string supportHeightTopic = "/entities/objectsHeight/top/";
    supportHeightTopic = supportHeightTopic + support1;
    node_->getParam(objectHeightTopic, objectHeight);
    node_->getParam(supportHeightTopic, supportHeight);
    toaster_msgs::ObjectListStamped objectList;

    double x1,y1,z, x2,y2;
    objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
     if(it->meEntity.id == support1){
        x1 = it->meEntity.pose.position.x;
        y1 = it->meEntity.pose.position.y;
        z = it->meEntity.pose.position.z;
     }
     if(it->meEntity.id == support2){
        x2 = it->meEntity.pose.position.x;
        y2 = it->meEntity.pose.position.y;
     }
    }
    z = z + objectHeight + supportHeight;
    toaster_msgs::SetEntityPose srv_setPose;
    srv_setPose.request.id = object;
    srv_setPose.request.type = "object";
    srv_setPose.request.pose.position.x = (x1 + x2)/2;
    srv_setPose.request.pose.position.y = (y1 + y2)/2;
    srv_setPose.request.pose.position.z = z;
    srv_setPose.request.pose.orientation.x = 0.0;
    srv_setPose.request.pose.orientation.y = 0.0;
    srv_setPose.request.pose.orientation.z = 0.0;
    srv_setPose.request.pose.orientation.w = 1.0;
    if (!client_set_pose_.call(srv_setPose)){
     ROS_ERROR("[human_monitor] Failed to call service pdg/set_entity_pose");
    }

    //we publish the action in the previous publisher
    supervisor_msgs::ActionsList msg_previous;
    action.succeed = true;
    msg_previous.actions.push_back(action);
    previous_pub_.publish(msg_previous);
}

/**
 * \brief Function which tells if an agent has already an object in hand
 * @param agent the agent tested
 * @return true if the agent has an object in hand, else false
 * */
std::pair<bool, std::string> HumanMonitor::hasInHand(std::string agent){
    std::pair<bool, std::string> answer;
    for(std::vector<std::pair<std::string, std::string> >::iterator it = attachments_.begin(); it != attachments_.end(); it++){
        if(it->first == agent){
            answer.first = true;
            answer.second = it->second;
            return answer;
        }
    }
    answer.first = false;
    return answer;
}

/**
 * \brief Function which tells if an object is a manipulable object (based on parameters)
 * @param object the object tested
 * @return true if the object is a manipulable object, else false
 * */
bool HumanMonitor::isManipulableObject(std::string object){

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = manipulableObjects_.begin(); it != manipulableObjects_.end(); it++){
      if(*it == object){
         return true;
      }
   }

   return false;

}

/**
 * \brief Function which tells if an object is a support object (based on parameters)
 * @param object the object tested
 * @return true if the object is a support object, else false
 * */
bool HumanMonitor::isSupportObject(std::string support){

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = supportObjects_.begin(); it != supportObjects_.end(); it++){
      if(*it == support){
         return true;
      }
   }

   return false;

}

/**
 * \brief Function which tells if an object is a container object (based on parameters)
 * @param object the object tested
 * @return true if the object is a container object, else false
 * */
bool HumanMonitor::isContainerObject(std::string container){

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = containerObjects_.begin(); it != containerObjects_.end(); it++){
      if(*it == container){
         return true;
      }
   }

   return false;

}
