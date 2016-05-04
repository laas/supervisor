/**
author Sandra Devin

Class allowing the execution of a pick action

**/

#include "action_executor/Actions/moveTo.h"

MoveTo::MoveTo(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 1){
        posName_ = action.parameters[0];
	}else{
		ROS_WARN("[action_executor] Wrong parameters number, should be: position");
	}
}

bool MoveTo::preconditions(){


   //We check if the position is valid
   string posTopic = "moveToPositions/";
   posTopic = posTopic + posName_;
   if(node_.hasParam(posTopic)){
       //we get the arm id
       string armTopic = "armPositions/";
       armTopic = armTopic + posName_;
       node_.getParam(armTopic, arm_);
       //we replace the position by the GTP name
       node_.getParam(posTopic, position_);
   }else{
      ROS_WARN("[action_executor] Wrong position name");
      return false;
   }
	
	return true;
 
}

bool MoveTo::plan(){

    if(arm_ == "right" && connector_->rightArmPose_ == posName_){
            return true;
    }else if(connector_->leftArmPose_ == posName_){
            return true;
    }

   vector<gtp_ros_msg::Ag> agents;
   gtp_ros_msg::Ag agent;
   agent.actionKey = "mainAgent";
   agent.agentName = "PR2_ROBOT";
   agents.push_back(agent);
   vector<gtp_ros_msg::Obj> objects;
   vector<gtp_ros_msg::Points> points;
   vector<gtp_ros_msg::Data> datas;
   gtp_ros_msg::Data data;
   data.dataKey = "confName";
   data.dataValue = position_;
   datas.push_back(data);
   
   actionId_ = planGTP("moveTo", agents, objects, datas, points);
   
   if(actionId_ == -1){
	   return false;
	}else{
	   return true;
	}
}

bool MoveTo::exec(Server* action_server){

    if(arm_ == "right" && connector_->rightArmPose_ == posName_){
            return true;
    }else if(connector_->leftArmPose_ == posName_){
            return true;
    }

    int armId;
    if(arm_ == "right"){
        armId = 0;
    }else{
        armId = 1;
    }

    return executeTrajectory(actionId_, 0, armId, action_server);

}

bool MoveTo::post(){

    if(arm_ == "right"){
        connector_->rightArmPose_ = posName_;
    }else{
        connector_->leftArmPose_ = posName_;
    }
	return true;
}
