/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/drop.h"

Drop::Drop(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		container_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, container");
    }
    lookAt(container_);
    connector->objectFocus_ = container_;
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = true;
}

bool Drop::preconditions(){

   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
       ROS_WARN("[action_executor] The object to place is not a known manipulable object");
      return false;
   }
   
   //First we check if the container is a known container object
   if(!isContainerObject(container_)){
       ROS_WARN("[action_executor] The container is not a known container object");
      return false;
   }
   
   //Then we check if the robot has the object in hand and if the object is reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
	fact.property = "isHoldBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
   fact.subjectId = container_;
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

bool Drop::plan(){

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
    gtp_ros_msg::Obj container;
    container.actionKey = "supportObject";
    container.objectName = container_;
    objects.push_back(container);
    vector<gtp_ros_msg::Points> points;
    vector<gtp_ros_msg::Data> datas;

    if(connector_->shouldUseRightHand_){
        gtp_ros_msg::Data data;
        data.dataKey = "hand";
        data.dataValue = "right";
        datas.push_back(data);
    }

    actionId_ = planGTP("drop", agents, objects, datas, points);

    if(actionId_ == -1){
        return false;
     }else{
        return true;
     }
}

bool Drop::exec(Server* action_server){

    return execAction(actionId_, false, action_server);

}

bool Drop::post(){

    PutInContainer(object_, container_);

	return true;
}
