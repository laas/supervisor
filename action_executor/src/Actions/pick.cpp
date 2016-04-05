/**
author Sandra Devin

Class allowing the execution of a pick action

**/

#include "action_executor/Actions/pick.h"

Pick::Pick(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 1){
		object_ = action.parameters[0];
	}else{
		ROS_WARN("[action_executor] Missing parameter: object to pick");
	}
}

bool Pick::preconditions(){
   
   //First we check if the object is a known manipulable object
   if(!isManipulableObject(object_)){
      ROS_WARN("[action_executor] The object to pick is not a known manipulable object");
      return false;
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
   
   actionId_ = planGTP("pick", agents, objects, datas, points);
   
   if(actionId_ == -1){
	   return false;
	}else{
	   return true;
	}

	return true;
}

bool Pick::exec(Server* action_server){

   return execAction(actionId_, true, action_server);

}

bool Pick::post(){

	return true;
}
