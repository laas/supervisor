/**
author Sandra Devin

Class allowing the execution of a pick and place action

**/

#include "action_executor/Actions/pickandplace.h"

PickAndPlace::PickAndPlace(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
		support_ = action.parameters[1];
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, support");
	}
}

bool PickAndPlace::preconditions(){

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

bool PickAndPlace::plan(){

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
    gtp_ros_msg::Obj support;
    support.actionKey = "supportObject";
    support.objectName = support_;
    objects.push_back(support);
    vector<gtp_ros_msg::Points> points;
    vector<gtp_ros_msg::Data> datas;
    string actionName;
    if(isManipulableObject(support_)){
        //if the support is also a manipulable object, this is a stack action
         actionName = "stack";
         //we add a point in order the objects to be align
         gtp_ros_msg::Points point;
         point.pointKey = "target";
         point.value.x = 0.0;
         point.value.y = 0.0;
         point.value.z = 0.0;
         points.push_back(point);
    }else{
        actionName = "place";
        string xParamTopic = "/points/";
        string yParamTopic = "/points/";
        string zParamTopic = "/points/";
        xParamTopic = xParamTopic + support_ + "/" + object_ + "/x";
        yParamTopic = xParamTopic + support_ + "/" + object_ + "/y";
        zParamTopic = xParamTopic + support_ + "/" + object_ + "/z";
        if(node_.hasParam(xParamTopic)){
            double pointX, pointY, pointZ;
            node_.getParam(xParamTopic, pointX);
            node_.getParam(yParamTopic, pointY);
            node_.getParam(zParamTopic, pointZ);
            double x, y, z;
            try{
                toaster_msgs::ObjectList objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectList>("pdg/objectList",ros::Duration(1)));
                for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
                  if(it->meEntity.id == support_){
                     x = it->meEntity.positionX;
                     y = it->meEntity.positionY;
                     z = it->meEntity.positionZ;
                     break;
                  }
                }
                x = x + pointX;
                y = y + pointY;
                z = z + pointZ;
                gtp_ros_msg::Points point;
                point.pointKey = "target";
                point.value.x = x;
                point.value.y = y;
                point.value.z = z;
                points.push_back(point);
            }
            catch(const std::exception & e){
                ROS_WARN("[action_executor] Failed to read %s pose from toaster", support_.c_str());
            }
       }
    }

    int nbTry = 0;
    while(nbTry < nbPlanMax_){
        actionId_ = planGTP("pick", agents, objects, datas, points);
        if(actionId_ == -1){
            return false;
         }else{
            int previousId = connector_->previousId_;
            connector_->previousId_ = actionId_;
            nextActionId_ = planGTP(actionName, agents, objects, datas, points);
            connector_->previousId_ = previousId;
            if(nextActionId_ != -1){
                return true;
            }
         }
        nbTry++;
    }

    return false;
}

bool PickAndPlace::exec(Server* action_server){

    bool firstTask = execAction(actionId_, true, action_server);

    if(firstTask){
        if(gripperEmpty_  && !simu_){
            ROS_WARN("[action_executor] Robot failed to pick (gripper empty)");
            return false;
        }
        //TODO: check gripper position (completly close or not)
        return execAction(nextActionId_, true, action_server);
    }else{
        return false;
    }

	return true;
}

bool PickAndPlace::post(){

    PutOnSupport(object_, support_);

	return true;
}
