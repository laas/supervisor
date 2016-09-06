/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/placereachable.h"

PlaceReachable::PlaceReachable(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
    if(action.parameters.size() == 3){
		object_ = action.parameters[0];
        objectRefined_ = isRefinedObject(object_);
        support_ = action.parameters[1];
        supportRefined_ = isRefinedObject(support_);
        targetAgent_ = action.parameters[2];
	}else{
        ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, support, targetAgent");
    }
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = true;
    originalAction_ = action;
}

bool PlaceReachable::preconditions(){

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

   if(!objectRefined_){
       ROS_WARN("[action_executor] The given object is not refined!");
      return false;
   }

   //If the support is not refined, we refine it
   if(!supportRefined_){
       bool uniqueSupport;
       if(isUniqueSupportObject(support_)){
           uniqueSupport = true;
       }else{
           uniqueSupport = false;
       }
       string refinedObject = refineObject(support_, uniqueSupport);
      if(refinedObject == "NULL"){
          ROS_WARN("[action_executor] No possible refinement for object: %s", support_.c_str());
          return false;
      }else{
           support_ = refinedObject;
      }
   }
   connector_->objectFocus_ = support_;
   connector_->objectToWatch_ = support_;
   
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

    //If there is no previous task we look for a previous grasp
    if(connector_->previousId_ == -1){
        if(connector_->idGrasp_ == -1){
            ROS_WARN("[action_executor] No previous Id nore previous grasp id");
           return false;
        }else{
            //TODO: add attachment in GTP
        }
    }

	return ArePreconditionsChecked(precsTocheck);

}

bool PlaceReachable::plan(){

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

    string replacementTopic = "/replacementPlacement/";
    replacementTopic = replacementTopic + support_;
    if(node_.hasParam(replacementTopic)){
        string replacementSupport;
        node_.getParam(replacementTopic, replacementSupport);
        gtp_ros_msg::Obj support;
        support.actionKey = "supportObject";
        support.objectName = replacementSupport;
        objects.push_back(support);
    }else{
        gtp_ros_msg::Obj support;
        support.actionKey = "supportObject";
        support.objectName = support_;
        objects.push_back(support);
    }

    if(connector_->shouldUseRightHand_){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = "right";
        datas.push_back(data);
    }

    actionId_ = planGTP("placereachable", agents, objects, datas, points);

    if(actionId_ == -1){
        return false;
     }else{
        return true;
     }
}

bool PlaceReachable::exec(Server* action_server){

    while(true){
        bool exec = execAction(actionId_, false, action_server);
        if(exec){
            return true;
        }else if(connector_->refineOrder_){
             connector_->objectLocked_ = connector_->objectToWatch_;
             string topic = "/highLevelName/";
             topic = topic + support_;
             node_.getParam(topic, support_);
             string refinedObject = refineObject(support_, false);
             if(refinedObject == "NULL"){
                 ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
                 return false;
             }else{
                  support_ = refinedObject;
                  connector_->objectFocus_ = support_;
                  connector_->objectToWatch_ = support_;
                  bool plan = this->plan();
                  if(!plan){
                      return false;
                  }
             }
        }else{
            return false;
        }
    }

}

bool PlaceReachable::post(){

   string replacementTopic = "/replacementPlacement/";
	replacementTopic = replacementTopic + support_;
	string replacementSupport;
	if(node_.hasParam(replacementTopic)){
		node_.getParam(replacementTopic, replacementSupport);
	}
	else{
		replacementSupport = support_;
	}
	PutOnSupport(object_, replacementSupport);

	return true;
}

supervisor_msgs::Action PlaceReachable::getInstantiatedAction(){

    supervisor_msgs::Action action = originalAction_;

    vector<string> newParams;
    newParams.push_back(object_);
    newParams.push_back(support_);
    newParams.push_back(targetAgent_);
    action.parameters = newParams;

    return action;
}
