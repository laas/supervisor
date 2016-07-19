/**
author Sandra Devin

Class allowing the execution of a pick and place action

**/

#include "action_executor/Actions/pickandplacereachable.h"

PickAndPlaceReachable::PickAndPlaceReachable(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
    if(action.parameters.size() == 3){
		object_ = action.parameters[0];
		support_ = action.parameters[1];
        targetAgent_ = action.parameters[2];
	}else{
        ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, support, targetAgent");
    }
    connector->objectFocus_ = object_;
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = true;

    lookAt(object_);
}

bool PickAndPlaceReachable::preconditions(){

   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
       ROS_WARN("[action_executor] The object to place is not a known manipulable object");
      return false;
   }
   
   //Then we check if the support is a known support object
   if(!isSupportObject(support_)){
       ROS_WARN("[action_executor] The support is not a known support object");
      return false;
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

bool PickAndPlaceReachable::plan(){

    //TO CHANGE
    connector_->previousId_ = -1;

    vector<gtp_ros_msg::Ag> agents;
    gtp_ros_msg::Ag agent;
    agent.actionKey = "mainAgent";
    agent.agentName = robotName_;
    agents.push_back(agent);
    gtp_ros_msg::Ag targetAgent;
    targetAgent.actionKey = "targetAgent";
    targetAgent.agentName = targetAgent_;
    agents.push_back(targetAgent);
    vector<gtp_ros_msg::Obj> objects;
    gtp_ros_msg::Obj object;
    object.actionKey = "mainObject";
    object.objectName = object_;
    objects.push_back(object);
    vector<gtp_ros_msg::Points> points;
    vector<gtp_ros_msg::Data> datas;

    if(connector_->shouldUseRightHand_){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = "right";
        datas.push_back(data);
    }

    string replacementTopic = "/replacementPlacementReachable/";
    replacementTopic = replacementTopic + support_;
    if(node_.hasParam(replacementTopic)){
        string replacementSupport;
        node_.getParam(replacementTopic, replacementSupport);
        gtp_ros_msg::Obj support;
        support.actionKey = "supportObject";
        support.objectName = replacementSupport;
        objects.push_back(support);
        ROS_INFO("param found topic: %s, object: %s", replacementTopic.c_str(), support.objectName.c_str());
    }else{
        gtp_ros_msg::Obj support;
        support.actionKey = "supportObject";
        support.objectName = support_;
        objects.push_back(support);
        ROS_INFO("param not found topic: %s, object: %s", replacementTopic.c_str(), support.objectName.c_str());
    }

    gtp_ros_msg::Points point;
    point.pointKey = "target";
    point.value.x = 0.0;
    point.value.y = 0.0;
    point.value.z = 0.0;
    points.push_back(point);

    int nbTry = 0;
    while(nbTry < nbPlanMax_){
        actionId_ = planGTP("pick", agents, objects, datas, points);
        if(actionId_ == -1){
            return false;
         }else{
            int previousId = connector_->previousId_;
            connector_->previousId_ = actionId_;
            nextActionId_ = planGTP("stack", agents, objects, datas, points);
            connector_->previousId_ = previousId;
            if(nextActionId_ != -1){
                return true;
            }
         }
        nbTry++;
    }

    return false;
}

bool PickAndPlaceReachable::exec(Server* action_server){

    bool firstTask = execAction(actionId_, true, action_server);

    if(firstTask){
        if(gripperEmpty_  && !simu_){
            ROS_WARN("[action_executor] Robot failed to pick (gripper empty)");
            return false;
        }
        connector_->objectFocus_ = support_;
        lookAt(support_);
        return execAction(nextActionId_, false, action_server);
    }else{
        return false;
    }

	return true;
}

bool PickAndPlaceReachable::post(){

    string replacementTopic = "/replacementPlacementReachable/";
    replacementTopic = replacementTopic + support_;
    string replacementSupport;
    if(node_.hasParam(replacementTopic)){
        node_.getParam(replacementTopic, replacementSupport);
    }else{
        replacementSupport = support_;
    }
    PutOnSupport(object_, replacementSupport);

	return true;
}
