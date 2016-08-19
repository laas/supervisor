/**
author Sandra Devin

Class allowing the execution of a place action

**/

#include "action_executor/Actions/pickanddrop.h"

PickAndDrop::PickAndDrop(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
	if(action.parameters.size() == 2){
		object_ = action.parameters[0];
        objectRefined_ = isRefinedObject(object_);
		container_ = action.parameters[1];
        containerRefined_ = isRefinedObject(container_);
	}else{
		ROS_WARN("[action_executor] Wrong parameter numbers, should be: object, container");
    }
    connector->objectFocus_ = object_;
    connector->weightFocus_ = 0.8;
    connector->stopableFocus_ = false;
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
   
   //If the object is not refined, we refine it
   if(!objectRefined_){
      string refinedObject = refineObject(object_);
      if(refinedObject == "NULL"){
          ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
          return false;
      }else{
           object_ = refinedObject;
      }
   }
   //TODO: add check of possible refinement for the container?

   //Then we check if  the object is reachable
   vector<toaster_msgs::Fact> precsTocheck;
   toaster_msgs::Fact fact;
   fact.subjectId = object_;
	fact.property = "isReachableBy";
	fact.targetId = robotName_;
    precsTocheck.push_back(fact);

	
	return ArePreconditionsChecked(precsTocheck);
}

bool PickAndDrop::plan(){

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

    int nbTry = 0;
    while(nbTry < nbPlanMax_){
        actionId_ = planGTP("pick", agents, objects, datas, points);
        if(actionId_ != -1){
            if(containerRefined_){
                int previousId = connector_->previousId_;
                connector_->previousId_ = actionId_;
                nextActionId_ = planGTP("drop", agents, objects, datas, points);
                connector_->previousId_ = previousId;
                if(nextActionId_ != -1){
                    return true;
                }
            }
         }
        nbTry++;
    }

    return false;
}

bool PickAndDrop::exec(Server* action_server){

    connector_->stopableFocus_ = true;
    bool firstTask = execAction(actionId_, true, action_server);

    if(firstTask){
        if(gripperEmpty_  && !simu_){
            ROS_WARN("[action_executor] Robot failed to pick (gripper empty)");
            return false;
        }
        //We refine the container if needed
        if(!containerRefined_){
            string refinedObject = refineObject(container_);
            if(refinedObject == "NULL"){
                ROS_WARN("[action_executor] No possible refinement for object: %s", container_.c_str());
                return false;
            }else{
                 container_ = refinedObject;
            }
        }
        connector_->objectFocus_ = container_;
        //we plan a traj if needed
        if(!containerRefined_){
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
            int previousId = connector_->previousId_;
            connector_->previousId_ = actionId_;
            int nbTry = 0;
            bool trajFound = false;
            while(nbTry < nbPlanMax_){
                nextActionId_ = planGTP("drop", agents, objects, datas, points);
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
        return execAction(nextActionId_, false, action_server);

    }else{
        return false;
    }
}

bool PickAndDrop::post(){

    PutInContainer(object_, container_);

	return true;
}
