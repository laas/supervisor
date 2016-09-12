/**
author Sandra Devin

Class allowing the execution of a pick and place action

**/

#include "action_executor/Actions/pickandplacereachable.h"

PickAndPlaceReachable::PickAndPlaceReachable(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
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
    connector->stopableFocus_ = false;
    originalAction_ = action;
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
   
   //If the object is not refined, we refine it
   if(!objectRefined_){
      string refinedObject = refineObject(object_, false);
      if(refinedObject == "NULL"){
          ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
          return false;
      }else{
           object_ = refinedObject;
      }
   }
   connector_->objectFocus_ = object_;
   connector_->objectToWatch_ = object_;
   //TODO: add check of possible refinement for the support?

   //Then we check if  the object is reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
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

    int nbTry = 0;
    while(nbTry < nbPlanMax_){
        actionId_ = planGTP("pick", agents, objects, datas, points);
        if(actionId_ != -1){
            int previousId = connector_->previousId_;
            connector_->previousId_ = actionId_;
            nextActionId_ = planGTP("placereachable", agents, objects, datas, points);
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

    connector_->stopableFocus_ = true;
    bool firstTask = false;
    while(!firstTask){
        bool exec = execAction(actionId_, true, action_server);
        if(exec){
            firstTask = true;
        }else if(connector_->refineOrder_){
             connector_->objectLocked_ = connector_->objectToWatch_;
             string topic = "/highLevelName/";
             topic = topic + object_;
             node_.getParam(topic, object_);
             string refinedObject = refineObject(object_, false);
             if(refinedObject == "NULL"){
                 ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
                 return false;
             }else{
                  object_ = refinedObject;
                  connector_->objectFocus_ = object_;
                  connector_->objectToWatch_ = object_;
                  bool plan = this->plan();
                  if(!plan){
                      return false;
                  }
             }
        }else{
            return false;
        }
    }

    if(firstTask){
        while(true){
        if(gripperEmpty_  && !simu_){
            ROS_WARN("[action_executor] Robot failed to pick (gripper empty)");
            return false;
        }
        //We refine the support if needed
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
        //we plan a traj if needed
        if(!supportRefined_){
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
            int previousId = connector_->previousId_;
            connector_->previousId_ = actionId_;
            int nbTry = 0;
            bool trajFound = false;
            while(nbTry < nbPlanMax_){
                nextActionId_ = planGTP("placereachable", agents, objects, datas, points);
                connector_->previousId_ = previousId;
                if(nextActionId_ != -1){
                    trajFound = true;
                    break;
                }
                nbTry++;
            }
            if(!trajFound){
                ROS_WARN("[action_executor] No traj found by GTP");
                return false;
            }
        }
        bool exec = execAction(nextActionId_, false, action_server);
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
             }
        }else{
            return false;
        }
    }
    }else{
        return false;
    }

	return true;
}

bool PickAndPlaceReachable::post(){

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

supervisor_msgs::Action PickAndPlaceReachable::getInstantiatedAction(){

    supervisor_msgs::Action action = originalAction_;

    vector<string> newParams;
    newParams.push_back(object_);
    newParams.push_back(support_);
    newParams.push_back(targetAgent_);
    action.parameters = newParams;

    return action;
}
