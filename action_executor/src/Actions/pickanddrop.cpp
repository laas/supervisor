/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/pickanddrop.h"

PickAndDrop::PickAndDrop(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		container_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, container");
	}
}

bool PickAndDrop::preconditions(){

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
   
   //Then we check if the container and the object are reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
   fact.subjectId = container_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
	precsTocheck.push_back(fact);
	
	return ArePreconditionsChecked(precsTocheck);
}

bool PickAndDrop::plan(){

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
    container.actionKey = "containerObject";
    container.objectName = container_;
    objects.push_back(container);
    vector<gtp_ros_msg::Points> points;
    vector<gtp_ros_msg::Data> datas;

    int nbTry = 0;
    while(nbTry < nbPlanMax_){
        actionId_ = planGTP("pick", agents, objects, datas, points);
        if(actionId_ == -1){
            return false;
         }else{
            int previousId = connector_->previousId_;
            connector_->previousId_ = actionId_;
            nextActionId_ = planGTP("drop", agents, objects, datas, points);
            connector_->previousId_ = previousId;
            if(nextActionId_ != -1){
                return true;
            }
         }
        nbTry++;
    }

    return false;
}

bool PickAndDrop::exec(Server* action_server){

    bool firstTask = execAction(actionId_, true, action_server);

    if(firstTask){
        //TODO: check gripper position (completly close or not)
        return execAction(nextActionId_, true, action_server);
    }else{
        return false;
    }
}

bool PickAndDrop::post(){

    PutInContainer(object_, container_);

	return true;
}
