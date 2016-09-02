/**
author Sandra Devin

Class allowing the execution of a pick action

**/

#include "action_executor/Actions/pick.h"

Pick::Pick(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 1){
		object_ = action.parameters[0];
        objectRefined_ = isRefinedObject(object_);
	}else{
		ROS_WARN("[action_executor] Missing parameter: object to pick");
    }
    connector->objectFocus_ = object_;
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = false;
    originalAction_ = action;
}

bool Pick::preconditions(){
   
    //First we check if the object is a known manipulable object
    if(!isManipulableObject(object_)){
      ROS_WARN("[action_executor] The object to pick is not a known manipulable object");
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

    //Then we check if the robot has the hands free and if the object is reachable
    vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = object_;
    fact.property = "isReachableBy";
    fact.targetId = robotName_;
    precsTocheck.push_back(fact);
	
	return ArePreconditionsChecked(precsTocheck);
 
}

bool Pick::plan(){

   //TO CHANGE
   connector_->previousId_ = -1;

   vector<gtp_ros_msg::Ag> agents;
   gtp_ros_msg::Ag agent;
   agent.actionKey = "mainAgent";
   agent.agentName = robotName_;
   agents.push_back(agent);
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
   
   actionId_ = planGTP("pick", agents, objects, datas, points);
   
   if(actionId_ == -1){
	   return false;
	}else{
	   return true;
	}

	return true;
}

bool Pick::exec(Server* action_server){

   connector_->stopableFocus_ = true;
   return execAction(actionId_, true, action_server);

}

bool Pick::post(){

    //Check gripper position (completly close or not)
    if(gripperEmpty_  && !simu_){
        ROS_WARN("[action_executor] Robot failed to pick (gripper empty)");
        return false;
    }

	return true;
}

supervisor_msgs::Action Pick::getInstantiatedAction(){

    supervisor_msgs::Action action = originalAction_;

    vector<string> newParams;
    newParams.push_back(object_);
    action.parameters = newParams;

    return action;
}
