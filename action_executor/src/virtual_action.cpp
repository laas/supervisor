/**
author Sandra Devin

Virtual class to represent an action, contain general function for the execution of actions

**/

#include <action_executor/virtual_action.h>

VirtualAction::VirtualAction(Connector* connector){

   node_.getParam("/simu", simu_);
   node_.getParam("/robot/name", robotName_);
   node_.getParam("/waitActionServer", waitActionServer_);
   node_.getParam("/nbPlanMaxGTP", nbPlanMax_);
   connector_ = connector;
   gripperEmpty_ = false;
}

/*
Function which return true if an object is not a high level object
    @object: the tested object
*/
bool VirtualAction::isRefinedObject(string object){

    string topic = "/highLevelRefinement/" + object;
    if(node_.hasParam(topic)){
        return false;
    }

   return true;

}

/*
Function which look for the refinement of an object
*/
string VirtualAction::refineObject(string object, bool uniqueSupport){

    //First we look for all possible refinement
    string topic = "/highLevelRefinement/" + object;
    vector<string> possibleObjects;
    node_.getParam(topic, possibleObjects);

    //Then, for all these objects we check if they are reachable by the robot
    double bestCost = 0.0;
    string objectStored = "NULL";
    for(vector<string>::iterator it = possibleObjects.begin(); it != possibleObjects.end(); it++){
        vector<toaster_msgs::Fact> factsTocheck;
        toaster_msgs::Fact fact;
        if(uniqueSupport){
            fact.subjectId = "NULL";
            fact.property = "isOn";
            fact.targetId = *it;
            factsTocheck.push_back(fact);
        }
        fact.subjectId = *it;
        fact.property = "isReachableBy";
        fact.targetId = robotName_;
        factsTocheck.push_back(fact);
        if(ArePreconditionsChecked(factsTocheck)){
            //TODO: add priority to the one not reachable by the human (objectStored and found if reachable by someone else)
            //get cost for this object (humanDistance)
            if(connector_->humanDistances_[*it] > bestCost){
                objectStored = *it;
            }
        }
    }

    return objectStored;
}

/*
Function which return true if an object is a manipulable object (based on parameters)
	@object: the tested object
*/
bool VirtualAction::isManipulableObject(string object){

   //first we get the manipulable object from parameters
   vector<string> manipulableObjects;
   node_.getParam("/entities/objects", manipulableObjects);
   
   //Then we check if the object is in the list
   for(vector<string>::iterator it = manipulableObjects.begin(); it != manipulableObjects.end(); it++){
      if(*it == object){
         return true;
      }
   }
   
   return false;
   
}

/*
Function which return true if an object is a support object (based on parameters)
	@support: the tested object
*/
bool VirtualAction::isSupportObject(string support){

   //first we get the manipulable object from parameters
   vector<string> supportObjects;
   node_.getParam("/entities/supports", supportObjects);
   
   //Then we check if the object is in the list
   for(vector<string>::iterator it = supportObjects.begin(); it != supportObjects.end(); it++){
      if(*it == support){
         return true;
      }
   }
   
   return false;
   
}

/*
Function which return true if an object is a support object (based on parameters)
    @support: the tested object
*/
bool VirtualAction::isUniqueSupportObject(string support){

   //first we get the manipulable object from parameters
   vector<string> supportObjects;
   node_.getParam("/uniqueSupports", supportObjects);

   //Then we check if the object is in the list
   for(vector<string>::iterator it = supportObjects.begin(); it != supportObjects.end(); it++){
      if(*it == support){
         return true;
      }
   }

   return false;

}


/*
Function which return true if an object is a container object (based on parameters)
	@container: the tested object
*/
bool VirtualAction::isContainerObject(string container){

   //first we get the manipulable object from parameters
   vector<string> containerObjects;
   node_.getParam("/entities/containers", containerObjects);
   
   //Then we check if the object is in the list
   for(vector<string>::iterator it = containerObjects.begin(); it != containerObjects.end(); it++){
      if(*it == container){
         return true;
      }
   }
   
   return false;
   
}

/*
Function which check if the preconditions are in the knowledge of the robot
	@precs: list of facts to check
*/
bool VirtualAction::ArePreconditionsChecked(vector<toaster_msgs::Fact> precs){

   ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
   supervisor_msgs::GetInfo srv;
	srv.request.agent = robotName_;
    srv.request.facts = precs;
    srv.request.info = "FACTS_IN";
	if (client.call(srv)){
        return srv.response.answer;
	}else{
       ROS_ERROR("[action_executor] Failed to call service mental_states/get_info");
	}
   return false;
}

/*
Function which puts an object in the hand of the robot
	@object: the object to put
	@hand: the hand to attach to (right or left)
*/
void VirtualAction::PutInHand(string object, string hand, int gtpId){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");

    //put the object in the hand of the robot
	string robotHand;
	string handTopic = "/robot/hands/";
	handTopic = handTopic + hand;
    node_.getParam(handTopic, robotHand);
    string robotToasterName;
    node_.getParam("robot/toasterName", robotToasterName);
	toaster_msgs::PutInHand srv;
   srv.request.objectId = object;
   srv.request.agentId = robotToasterName;
   srv.request.jointName = robotHand;
   if (!client.call(srv)){
   	 ROS_ERROR("[action_executor] Failed to call service pdg/put_in_hand");
 	}
   //remember the gtp id of the grasp
   connector_->idGrasp_ = gtpId;
   
}

/*
Function which remove an object from the hand of the robot
	@object: the object to remove
*/
void VirtualAction::RemoveFromHand(string object){

   ros::ServiceClient client = node_.serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");

    //remove the object from the hand of the robot
	toaster_msgs::RemoveFromHand srv;
   srv.request.objectId = object;
   if (!client.call(srv)){
   	 ROS_ERROR("[action_executor] Failed to call service pdg/remove_from_hand");
 	}
   
}

bool VirtualAction::isGripperEmpty(string arm){

    string gripperJoint;
    double gripperThreshold;
    string gripperTopic = "/gripperJoint/";
    gripperTopic = gripperTopic + arm;
    node_.getParam(gripperTopic, gripperJoint);
    node_.getParam("/gripperThreshold", gripperThreshold);
    toaster_msgs::RobotListStamped list;
    try{
        list  = *(ros::topic::waitForMessage<toaster_msgs::RobotListStamped>("pdg/robotList",ros::Duration(1)));
        for(vector<toaster_msgs::Robot>::iterator it = list.robotList.begin(); it != list.robotList.end(); it++){
          if(it->meAgent.meEntity.id == robotName_){
              for(vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); itj++){
                  if(itj->meEntity.id == gripperJoint){
                      if(itj->position < gripperThreshold){
                          return true;
                      }else{
                          return false;
                      }
                  }
              }
          }
        }
    }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read robot pose from toaster");
    }
    return true;
}


/*
Function which puts an object on a support
	@object: the object to put
    @support
*/
void VirtualAction::PutOnSupport(string object, string support){

   ros::ServiceClient client;
   if(connector_->simu_){
    client = node_.serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");
   }else{
    client = node_.serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
   }

   double objectHeight, supportHeight;
	string objectHeightTopic = "/objectsHeight/bottom/";
	objectHeightTopic = objectHeightTopic + object;
	string supportHeightTopic = "/objectsHeight/top/";
	supportHeightTopic = supportHeightTopic + support;
	node_.getParam(objectHeightTopic, objectHeight);
	node_.getParam(supportHeightTopic, supportHeight);
    toaster_msgs::ObjectListStamped objectList;
	double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == support){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
       z = z + objectHeight + supportHeight;
       toaster_msgs::SetEntityPose srv;
       srv.request.id = object;
       srv.request.type = "object";
       srv.request.pose.position.x = x;
       srv.request.pose.position.y = y;
       srv.request.pose.position.z = z;
       srv.request.pose.orientation.x = 0.0;
       srv.request.pose.orientation.y = 0.0;
       srv.request.pose.orientation.z = 0.0;
       srv.request.pose.orientation.w = 1.0;
       if (!client.call(srv)){
         ROS_ERROR("Failed to call service pdg/set_entity_pose");
    	 }
   }
   catch(const std::exception & e){
       ROS_WARN("[action_executor] Failed to read %s pose from toaster", support.c_str());
   }
   
}

/*
Function which puts an object in a container
    @object: the object to put
    @container
*/
void VirtualAction::PutInContainer(string object, string container){

    ros::ServiceClient client;
    if(connector_->simu_){
     client = node_.serviceClient<toaster_msgs::SetEntityPose>("toaster_simu/set_entity_pose");
    }else{
     client = node_.serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    }

    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
   try{
       objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
       for(vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
         if(it->meEntity.id == container){
            x = it->meEntity.pose.position.x;
            y = it->meEntity.pose.position.y;
            z = it->meEntity.pose.position.z;
            break;
         }
       }
       toaster_msgs::SetEntityPose srv;
       srv.request.id = object;
       srv.request.type = "object";
       srv.request.pose.position.x = x;
       srv.request.pose.position.y = y;
       srv.request.pose.position.z = z;
       srv.request.pose.orientation.x = 0.0;
       srv.request.pose.orientation.y = 0.0;
       srv.request.pose.orientation.z = 0.0;
       srv.request.pose.orientation.w = 1.0;
       if (!client.call(srv)){
         ROS_ERROR("Failed to call service pdg/set_entity_pose");
         }
   }
   catch(const std::exception & e){
       ROS_WARN("[action_executor] Failed to read %s pose from toaster", container.c_str());
   }

}

/*
Function which update GTP world state with TOASTER
*/
bool VirtualAction::updateGTP(){
   
  // send goal to GTP
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "update";
  connector_->acGTP_->sendGoal(goal);

  //wait for the action to return
  bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(waitActionServer_));

  if (!finishedBeforeTimeout){
    ROS_INFO("[action_executor] GTP Action did not finish before the time out.");
    return false;
  }
   
   return true;
}

/*
Function which ask a plan to GTP and return the id of the solution (-1 if no solution)
*/
int VirtualAction::planGTP(string actionName, vector<gtp_ros_msg::Ag> agents, vector<gtp_ros_msg::Obj> objects, vector<gtp_ros_msg::Data> datas, vector<gtp_ros_msg::Points> points){
  
  updateGTP();
  
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "planning";
  goal.req.actionName = actionName;
  goal.req.involvedAgents = agents;
  goal.req.involvedObjects = objects;
  goal.req.data = datas;
  goal.req.points = points;
  goal.req.predecessorId.actionId = connector_->previousId_;
  goal.req.predecessorId.alternativeId = 0;

  int nbTry = 0;
  while(nbTry < nbPlanMax_){
     connector_->acGTP_->sendGoal(goal);
     bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(waitActionServer_));

     if (finishedBeforeTimeout)
     {
       if(connector_->acGTP_->getResult()->ans.success){
           return connector_->acGTP_->getResult()->ans.identifier.actionId;
       }
     }
     else{
         ROS_INFO("[action_executor] GTP Action did not finish before the time out.");
         return -1;
     }
       
      nbTry++;
   }
   
   return -1;
}

/*
Function which execute an action based on its GTP id
*/
bool VirtualAction::execAction(int actionId, bool shouldOpen, Server* action_server){
  
  gtp_ros_msg::requestGoal goal;
  goal.req.requestType = "details";
  goal.req.loadAction.actionId = actionId;
  goal.req.loadAction.alternativeId = 0;
  
  connector_->acGTP_->sendGoal(goal);
  bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(waitActionServer_));

  if (finishedBeforeTimeout){
     vector<gtp_ros_msg::SubTraj> subTrajs = connector_->acGTP_->getResult()->ans.subTrajs;
     if(shouldOpen && ((subTrajs[0].armId== 0 && !connector_->gripperRightOpen_) || (subTrajs[0].armId== 1 && !connector_->gripperLeftOpen_))){//the robot should have the gripper open to execute the trajectory
        openGripper(subTrajs[0].armId, action_server);
     }
     for(vector<gtp_ros_msg::SubTraj>::iterator it = subTrajs.begin(); it != subTrajs.end(); it++){
         if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_){
            return false;
         }
         if(it->agent == robotName_){
            if(it->subTrajName == "grasp"){
                closeGripper(it->armId, action_server);
                string hand;
                if(it->armId == 0){
                    hand = "right";
                }else{
                    hand = "left";
                }
                if(!gripperEmpty_  || simu_){
                    PutInHand(object_, hand, actionId);
                }
            }else if(it->subTrajName == "release"){
                openGripper(it->armId, action_server);
                RemoveFromHand(object_);
            }else{//this is a trajectory
                executeTrajectory(actionId, it->subTrajId, it->armId, action_server);
            }
         }
     }
  }else{
     ROS_INFO("[action_executor] GTP Action did not finish before the time out.");
     return false;
  }
  
  connector_->previousId_ = actionId;
  return true;
  
}


/*
Function which execute a trajectory
*/
bool VirtualAction::executeTrajectory(int actionId, int actionSubId, int armId, Server* action_server){

   gtp_ros_msg::requestGoal goal;
   goal.req.requestType = "load";
   goal.req.loadAction.actionId = actionId;
   goal.req.loadAction.alternativeId = 0;
   goal.req.loadSubTraj = actionSubId;

   connector_->acGTP_->sendGoal(goal);
   bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(waitActionServer_));

   if (finishedBeforeTimeout){
     if(armId == 0){//right arm
        pr2motion::Arm_Right_MoveGoal arm_goal_right;
        arm_goal_right.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        arm_goal_right.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
        connector_->rightArmMoving_ = true;
        connector_->PR2motion_arm_right_->sendGoal(arm_goal_right,  boost::bind(&VirtualAction::moveRightArm, this, _1, _2),
                                                   Client_Right_Arm::SimpleActiveCallback(),  Client_Right_Arm::SimpleFeedbackCallback());
        while(connector_->rightArmMoving_ == true){
            //wait for preempted request or end of the action
            if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_){
                connector_->PR2motion_arm_right_->cancelGoal();
                return false;
            }
        }
        connector_->rightArmPose_ = "unknown";
     }else{
        pr2motion::Arm_Left_MoveGoal arm_goal_left;
        arm_goal_left.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        arm_goal_left.path_mode.value=pr2motion::pr2motion_PATH_MODE::pr2motion_PATH_PORT;
        connector_->leftArmMoving_ = true;
        connector_->PR2motion_arm_left_->sendGoal(arm_goal_left,  boost::bind(&VirtualAction::moveLeftArm, this, _1, _2),
                                                   Client_Left_Arm::SimpleActiveCallback(),  Client_Left_Arm::SimpleFeedbackCallback());
        while(connector_->leftArmMoving_ == true){
            //wait for preempted request or end of the action
            if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_){
                connector_->PR2motion_arm_left_->cancelGoal();
                return false;
            }
        }
        connector_->leftArmPose_ = "unknown";

     }
   }
   else{
    ROS_INFO("[action_executor] GTP Action did not finish before the time out.");
    return false;
   }
   
   return true;

}


/*
Function which close a gripper
*/
bool VirtualAction::closeGripper(int armId, Server* action_server){

    bool finishedBeforeTimeout;
    if(armId == 0){//right arm
       pr2motion::Gripper_Right_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
       connector_->rightGripperMoving_ = true;
       connector_->PR2motion_gripper_right_->sendGoal(gripper_goal,  boost::bind(&VirtualAction::moveRightGripper, this, _1, _2),
                                                  Client_Right_Gripper::SimpleActiveCallback(),  Client_Right_Gripper::SimpleFeedbackCallback());
       while(connector_->rightGripperMoving_ == true){
           //wait for preempted request or end of the action
           if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_){
               connector_->PR2motion_gripper_right_->cancelGoal();
               return false;
           }
       }
       gripperEmpty_  = isGripperEmpty("right");
    }else{
       pr2motion::Gripper_Left_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_CLOSE;
       connector_->leftGripperMoving_ = true;
       connector_->PR2motion_gripper_left_->sendGoal(gripper_goal,  boost::bind(&VirtualAction::moveLeftGripper, this, _1, _2),
                                                  Client_Left_Gripper::SimpleActiveCallback(),  Client_Left_Gripper::SimpleFeedbackCallback());
       while(connector_->leftGripperMoving_ == true){
           //wait for preempted request or end of the action
           if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_){
               connector_->PR2motion_gripper_left_->cancelGoal();
               return false;
           }
       }
       gripperEmpty_  = isGripperEmpty("left");
    }


   return true;

}


/*
Function which open a gripper
*/
bool VirtualAction::openGripper(int armId, Server* action_server){

    bool finishedBeforeTimeout;
    if(armId == 0){//right arm
       pr2motion::Gripper_Right_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       connector_->rightGripperMoving_ = true;
       connector_->PR2motion_gripper_right_->sendGoal(gripper_goal);
       finishedBeforeTimeout = connector_->PR2motion_gripper_right_->waitForResult(ros::Duration(waitActionServer_));
       connector_->rightGripperMoving_ = false;
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
         return false;
       }
       connector_->gripperRightOpen_= true;
    }else{
       pr2motion::Gripper_Left_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       connector_->leftGripperMoving_ = true;
       connector_->PR2motion_gripper_left_->sendGoal(gripper_goal);
       finishedBeforeTimeout = connector_->PR2motion_gripper_left_->waitForResult(ros::Duration(waitActionServer_));
       connector_->leftGripperMoving_ = false;
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
         return false;
       }
       connector_->gripperLeftOpen_= true;
    }

   return true;

}

/*
Called once when the goal of the right arm action client completes
*/
void VirtualAction::moveRightArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Right_MoveResultConstPtr& result){

        connector_->rightArmMoving_ = false;
}

/*
Called once when the goal of the left arm action client completes
*/
void VirtualAction::moveLeftArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Left_MoveResultConstPtr& result){

        connector_->leftArmMoving_ = false;
}

/*
Called once when the goal of the right gripper action client completes
*/
void VirtualAction::moveRightGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Right_OperateResultConstPtr& result){

        connector_->rightGripperMoving_ = false;
}

/*
Called once when the goal of the left gripper action client completes
*/
void VirtualAction::moveLeftGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Left_OperateResultConstPtr& result){

        connector_->leftGripperMoving_ = false;
}

