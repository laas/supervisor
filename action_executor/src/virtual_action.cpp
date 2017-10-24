/**
author Sandra Devin
Virtual class to represent an action, contain general function for the execution of actions
**/

#include <action_executor/virtual_action.h>


/**
 * \brief Construction of the class
 * @param connector pointer to the connector structure
 * */
VirtualAction::VirtualAction(Connector* connector){

   connector_ = connector;
   gripperEmpty_ = false;

   initialObject_ = "NULL";
}

/**
 * \brief Function which return true if an object is a manipulable object (based on parameters)
 * @param object the tested object
 * @return true is the object is a manipulable object
 * */
bool VirtualAction::isManipulableObject(std::string object){

   //first we get the manipulable objects from parameters
   std::vector<std::string> manipulableObjects;
   connector_->node_->getParam("/entities/objects", manipulableObjects);

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = manipulableObjects.begin(); it != manipulableObjects.end(); it++){
      if(*it == object){
         return true;
      }
   }

   return false;

}

/**
 * \brief Function which return true if an object is a support object (based on parameters)
 * @param support the tested object
 * @return true is the object is a support
 * */
bool VirtualAction::isSupportObject(std::string support){

   //first we get the support objects from parameters
   std::vector<std::string> supportObjects;
   connector_->node_->getParam("/entities/supports", supportObjects);

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = supportObjects.begin(); it != supportObjects.end(); it++){
      if(*it == support){
         return true;
      }
   }

   return false;

}

/**
 * \brief Function which return true if an object is a container object (based on parameters)
 * @param container the tested object
 * @return true is the object is a container
 * */
bool VirtualAction::isContainerObject(std::string container){

   //first we get the container objects from parameters
   std::vector<std::string> containerObjects;
   connector_->node_->getParam("/entities/containers", containerObjects);

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = containerObjects.begin(); it != containerObjects.end(); it++){
      if(*it == container){
         return true;
      }
   }

   return false;

}

/**
 * \brief Function which return true if an object is a unique support (based on parameters)
 * @param support the tested object
 * @return true is the object is a unique support
 * */
bool VirtualAction::isUniqueSupport(std::string support){

   //first we get the unique supports from parameters
   std::vector<std::string> uniqueSupports;
   connector_->node_->getParam("/action_executor/uniqueSupports", uniqueSupports);

   //Then we check if the object is in the list
   for(std::vector<std::string>::iterator it = uniqueSupports.begin(); it != uniqueSupports.end(); it++){
      if(*it == support){
         return true;
      }
   }

   return false;

}

/**
 * \brief Function which check if the preconditions are in the knowledge of the robot
 * @param precs list of facts to check
 * @return true if the preconditions are in the db
 * */
bool VirtualAction::ArePreconditionsChecked(std::vector<toaster_msgs::Fact> precs){

    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.agent = connector_->robotName_;
    srv.request.facts = precs;
    if (connector_->client_db_execute_.call(srv)){
        return srv.response.boolAnswer;
    }else{
       ROS_ERROR("[action_executor] Failed to call service database_manager/execute");
    }
    return false;
}

/**
 * \brief Function which puts an object in the hand of the robot
 * @param object the object to put in hand
 * @param hand the hand to attach to
 * */
void VirtualAction::PutInHand(std::string object, std::string hand, int gtpId){

    //add the fact that the object is in the robot hand
    std::vector<toaster_msgs::Fact> toAdd;
    toaster_msgs::Fact fact;
    fact.subjectId = object;
    fact.property = "isHoldBy";
    fact.targetId = connector_->robotName_;
    fact.factObservability = 1.0;
    toAdd.push_back(fact);

    toaster_msgs::SetInfoDB srv_fact;
    srv_fact.request.agentId = connector_->robotName_;
    srv_fact.request.facts = toAdd;
    srv_fact.request.infoType = "FACT";
    srv_fact.request.add = true;
    if(!connector_->client_db_set_.call(srv_fact)){
        ROS_ERROR("[action_executor] Failed to call service database_manager/set_info");
    }

    std::vector<toaster_msgs::Fact> toRm;
    fact.property = "isOn";
    fact.targetId = "NULL";
    toRm.push_back(fact);

    srv_fact.request.agentId = connector_->robotName_;
    srv_fact.request.facts = toRm;
    srv_fact.request.add = false;
    if(!connector_->client_db_set_.call(srv_fact)){
        ROS_ERROR("[action_executor] Failed to call service database_manager/set_info");
    }


    //put the object in the hand of the robot
    std::string robotHand;
    std::string handTopic = "/supervisor/robot/hand/" + hand;
    connector_->node_->getParam(handTopic, robotHand);
    toaster_msgs::PutInHand srv;
    srv.request.objectId = object;
    srv.request.agentId = connector_->robotToaster_;
    srv.request.jointName = robotHand;
    if (!connector_->client_put_hand_.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service pdg/put_in_hand");
    }
    //remember the gtp id of the grasp
    connector_->idGrasp_ = gtpId;

}

/**
 * \brief Function which remove an object from the hand of the robot
 * @param object the object to remove
 * */
void VirtualAction::RemoveFromHand(std::string object){

    //remove the fact that the object is in the robot hand
    std::vector<toaster_msgs::Fact> toRm;
    toaster_msgs::Fact fact;
    fact.subjectId = object;
    fact.property = "isHoldBy";
    fact.targetId = "NULL";
    toRm.push_back(fact);

    toaster_msgs::SetInfoDB srv_fact;
    srv_fact.request.agentId = connector_->robotName_;
    srv_fact.request.facts = toRm;
    srv_fact.request.infoType = "FACT";
    srv_fact.request.add = false;
    if(!connector_->client_db_set_.call(srv_fact)){
        ROS_ERROR("[action_executor] Failed to call service database_manager/set_info");
    }

    //remove the object from the hand of the robot
    toaster_msgs::RemoveFromHand srv;
    srv.request.objectId = object;
    if (!connector_->client_remove_hand_.call(srv)){
     ROS_ERROR("[action_executor] Failed to call service pdg/remove_from_hand");
    }

}

/**
 * \brief Function which tells if the robot gripper is empty
 * @param arm the arm of the gripper to test
 * @return true if the gripper is empty
 * */
bool VirtualAction::isGripperEmpty(std::string arm){

    //check the robot gripper position (if it is too much closed, it is empty)
    std::string gripperJoint;
    double gripperThreshold;
    std::string gripperTopic = "/supervisor/robot/gripperJoint/"+ arm;
    connector_->node_->getParam(gripperTopic, gripperJoint);
    connector_->node_->getParam("/action_executor/gripperThreshold", gripperThreshold);
    toaster_msgs::RobotListStamped list;
    list  = *(ros::topic::waitForMessage<toaster_msgs::RobotListStamped>("pdg/robotList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Robot>::iterator it = list.robotList.begin(); it != list.robotList.end(); it++){
      if(it->meAgent.meEntity.id == connector_->robotName_){
          for(std::vector<toaster_msgs::Joint>::iterator itj = it->meAgent.skeletonJoint.begin(); itj != it->meAgent.skeletonJoint.end(); itj++){
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

    return true;
}


/**
 * \brief Function which puts an object on a support
 * @param object the object to put
 * @param support the support where to place
 * */
void VirtualAction::PutOnSupport(std::string object, std::string support){

    double objectHeight, supportHeight;
    std::string objectHeightTopic = "/entities/objectsHeight/bottom/" + object;
    std::string supportHeightTopic = "/entities/objectsHeight/top/" + support;
    connector_->node_->getParam(objectHeightTopic, objectHeight);
    connector_->node_->getParam(supportHeightTopic, supportHeight);
    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
    objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
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
    if (!connector_->client_set_pose_.call(srv)){
     ROS_ERROR("Failed to call service toaster_simu/set_entity_pose");
    }

}

/**
 * \brief Function which puts an object in a container
 * @param object the object to put
 * @param container the container where to place
 * */
void VirtualAction::PutInContainer(std::string object, std::string container){

    toaster_msgs::ObjectListStamped objectList;
    double x,y,z;
    objectList  = *(ros::topic::waitForMessage<toaster_msgs::ObjectListStamped>("pdg/objectList",ros::Duration(1)));
    for(std::vector<toaster_msgs::Object>::iterator it = objectList.objectList.begin(); it != objectList.objectList.end(); it++){
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
    if (!connector_->client_set_pose_.call(srv)){
     ROS_ERROR("Failed to call service pdg/set_entity_pose");
    }

}


/**
 * \brief Function which ask a plan to GTP
 * @param actionName name of the action to plan
 * @param agents agents invloved
 * @param objects objects invloved
 * @param datas datas invloved
 * @param points points invloved
 * @return the gtp id of the result (-1 if no result) and the corresponding subsolutions
 * */
std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > VirtualAction::planGTP(std::string actionName, std::vector<gtp_ros_msgs::Role> agents, std::vector<gtp_ros_msgs::Role> objects, std::vector<gtp_ros_msgs::MiscData> datas, std::vector<gtp_ros_msgs::Point> points, std::vector<gtp_ros_msgs::ActionId> attachments){

    std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > res;

    if(connector_->noPlanning_){
        res.first = 0;
        return res;
    }

    res.first = -1;

  gtp_ros_msgs::PlanGoal goal;
  goal.request.taskType = actionName;
  goal.request.agents = agents;
  goal.request.objects = objects;
  goal.request.data = datas;
  goal.request.points = points;
  goal.request.previousAction.taskId = connector_->previousId_;
  goal.request.previousAction.alternativeId = 0;
  goal.request.computeMotionPlan = true;
  goal.request.setAttachmentsFrom = attachments;
  if(connector_->previousId_  == -1){
      goal.request.updateBefore = true;
  }else{
      goal.request.updateBefore = false;
  }

  int nbTry = 0;
  while(nbTry < connector_->nbPlanMax_){
     connector_->acGTP_->sendGoal(goal);
     bool finishedBeforeTimeout = connector_->acGTP_->waitForResult(ros::Duration(connector_->waitActionServer_));
     if (finishedBeforeTimeout) {
       if(connector_->acGTP_->getResult()->result.success){
           res.first = connector_->acGTP_->getResult()->result.id.taskId;
           res.second = connector_->acGTP_->getResult()->result.solutionParts;
           return res;
       }
     }
     else{
         ROS_INFO("[action_executor] GTP Action did not finish before the time out.");
         return res;
     }

      nbTry++;
   }

   return res;
}

/**
 * \brief Function which execute an action based on its GTP id
 * @param actionId the gtp id
 * @param shouldOpen should open the gripper before execution
 * @param action_server pointer to the action server
 * @return true if success
 * */
bool VirtualAction::execAction(int actionId, std::vector<gtp_ros_msgs::SubSolution> subSolutions, bool shouldOpen, Server* action_server){

    if(connector_->noPlanning_){
        return true;
    }

    if(connector_->saveMode_ == "save"){
        connector_->fileSave_ <<"  "  << actionName_ << "_" << param1_ << "_" << param2_ << ":" << std::endl;
    }
    
    //the robot should have the gripper open to execute the trajectory
    if(shouldOpen && ((subSolutions[0].armId== 0 && !connector_->gripperRightOpen_) || (subSolutions[0].armId== 1 && !connector_->gripperLeftOpen_))){
        openGripper(subSolutions[0].armId, action_server);
    }

    for(std::vector<gtp_ros_msgs::SubSolution>::iterator it = subSolutions.begin(); it != subSolutions.end(); it++){
     if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_){
        return false;
     }
     if(it->agent == connector_->robotName_){
        if(it->name == "grasp"){
            closeGripper(it->armId, action_server);
            std::string hand;
            if(it->armId == 0){
                hand = "right";
            }else{
                hand = "left";
            }
            if(!gripperEmpty_  || connector_->simu_){
                PutInHand(object_, hand, actionId);
            }
        }else if(it->name == "release"){
            openGripper(it->armId, action_server);
            RemoveFromHand(object_);
        }else{//this is a trajectory
            executeTrajectory(actionId, it->id, it->armId, action_server, it->name);
        }
     }
    }

    connector_->previousId_ = actionId;
    return true;

}

/**
 * \brief Function which execute a trajectory
 * @param actionId the gtp id of the task
 * @param actionSubId the gtp id of the trajectory
 * @param armId the arm used
 * @param action_server pointer to the action server
 * @return true if success
 * */
bool VirtualAction::executeTrajectory(int actionId, int actionSubId, int armId, Server* action_server, std::string name){

   if (connector_->noExec_){
       //no execution required
       return true;
   }

   if(connector_->saveMode_ == "load" && actionName_ != "moveTo"){
        //we get the traj from param
       gtp_ros_msg::GTPTraj traj;
       traj.name = name;
       std::string baseTopic = "trajs/" + actionName_ + "_" + param1_ + "_" + param2_ + "/" + name + "/";
       std::vector<std::string> joint_names;
       std::string jointNamesTopic = baseTopic + "joint_names";
       if(!connector_->node_->hasParam(jointNamesTopic)){
           ROS_WARN("Traj not found in param: %s", baseTopic.c_str());
           return false;
       }
       connector_->node_->getParam(jointNamesTopic, joint_names);
       std::string nbPointsTopic = baseTopic + "nbPoints";
       int nbPoints;
       connector_->node_->getParam(nbPointsTopic, nbPoints);
       for(std::vector<std::string>::iterator it = joint_names.begin(); it != joint_names.end(); it++){
           traj.traj.joint_names.push_back(*it);
       }
       for(int i = 0; i < nbPoints; i++){
           trajectory_msgs::JointTrajectoryPoint point;
           std::stringstream ss;
           ss << i;
           std::string strI = ss.str();
           std::string positionTopic = baseTopic + "point_" + strI + "/positions";
           connector_->node_->getParam(positionTopic, point.positions);
           std::string velocitiesTopic = baseTopic + "point_" + strI + "/velocities";
           connector_->node_->getParam(velocitiesTopic, point.velocities);
           std::string accelTopic = baseTopic + "point_" + strI + "/accelerations";
           connector_->node_->getParam(accelTopic, point.accelerations);
           std::string effortTopic = baseTopic + "point_" + strI + "/effort";
           connector_->node_->getParam(effortTopic, point.effort);
           traj.traj.points.push_back(point);
       }
       //we publish it in the corresponding topic
       connector_->gtp_pub_.publish(traj);
   }else{
       if(connector_->saveMode_ == "save" && actionName_ != "moveTo"){
           connector_->needTraj_ = true;
       }
       gtp_ros_msgs::PublishTraj srv;
       srv.request.actionId.taskId = actionId;
       srv.request.actionId.alternativeId = 0;
       srv.request.subSolutionId = actionSubId;
       if (!connector_->client_gtp_traj_.call(srv)){
           ROS_ERROR("[action_executor] Failed to call gtp/publishTraj");
           return false;
       }
   }


   //if needed, save the traj
   if(connector_->saveMode_ == "save"){
       ROS_INFO("Waiting for gtp traj to save");
       while(connector_->needTraj_){

       }
       ROS_INFO("Gtp traj received");
       saveTraj(connector_->curTraj_, name);
   }
   if(armId == 0){//right arm
      pr2motion::Arm_Right_MoveGoal arm_goal_right;
      if(name == "engage" || name == "escape"){
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
   }else{
      ROS_ERROR("[action_executor] Failed to call gtp/publishTraj");
      return false;
   }


   return true;

}


/**
 * \brief Function which close a gripper
 * @param armId the arm of the gripper to close
 * @param action_server pointer to the action server
 * @return true if success
 * */
bool VirtualAction::closeGripper(int armId, Server* action_server){

    if (connector_->noExec_){
        //no execution required
        return true;
    }

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
           if(action_server->isPreemptRequested() || connector_->stopOrder_ || connector_->refineOrder_    ){
               connector_->PR2motion_gripper_left_->cancelGoal();
               return false;
           }
       }
       gripperEmpty_  = isGripperEmpty("left");
    }


   return true;

}


/**
 * \brief Function which open a gripper
 * @param armId the arm of the gripper to open
 * @param action_server pointer to the action server
 * @return true if success
 * */
bool VirtualAction::openGripper(int armId, Server* action_server){

    if (connector_->noExec_){
        //no execution required
        return true;
    }

    bool finishedBeforeTimeout;
    if(armId == 0){//right arm
       pr2motion::Gripper_Right_OperateGoal gripper_goal;
       gripper_goal.goal_mode.value=pr2motion::pr2motion_GRIPPER_MODE::pr2motion_GRIPPER_OPEN;
       connector_->rightGripperMoving_ = true;
       connector_->PR2motion_gripper_right_->sendGoal(gripper_goal);
       finishedBeforeTimeout = connector_->PR2motion_gripper_right_->waitForResult(ros::Duration(connector_->waitActionServer_));
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
       finishedBeforeTimeout = connector_->PR2motion_gripper_left_->waitForResult(ros::Duration(connector_->waitActionServer_));
       connector_->leftGripperMoving_ = false;
       if(!finishedBeforeTimeout){
         ROS_INFO("[action_executor] PR2motion Action did not finish before the time out.");
         return false;
       }
       connector_->gripperLeftOpen_= true;
    }

   return true;

}

/**
 * \brief Called once when the goal of the right arm action client completes
 * @param state state of the action server
 * @param result result of the action server
 * */
void VirtualAction::moveRightArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Right_MoveResultConstPtr& result){

        connector_->rightArmMoving_ = false;
}

/**
 * \brief Called once when the goal of the left arm action client completes
 * @param state state of the action server
 * @param result result of the action server
 * */
void VirtualAction::moveLeftArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Left_MoveResultConstPtr& result){

        connector_->leftArmMoving_ = false;
}

/**
 * \brief Called once when the goal of the right gripper action client completes
 * @param state state of the action server
 * @param result result of the action server
 * */
void VirtualAction::moveRightGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Right_OperateResultConstPtr& result){

        connector_->rightGripperMoving_ = false;
}

/**
 * \brief Called once when the goal of the left gripper action client completes
 * @param state state of the action server
 * @param result result of the action server
 * */
void VirtualAction::moveLeftGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Left_OperateResultConstPtr& result){

        connector_->leftGripperMoving_ = false;
}

/**
 * \brief Function which return true if an object is not a high level object
 * @param object the tested object
 * @return true is not a high level object
 * */
bool VirtualAction::isRefined(std::string object){

   if(connector_->highLevelRefinment_[object].size() > 0){
       return false;
   }

   return true;

}

/**
 * \brief Function which look for a refinment for an object
 * @param object the high level object
 * @param conditions facts the object need to respect
 * @return the new object or NULL if not
 * */
std::string VirtualAction::findRefinment(std::string object, std::vector<toaster_msgs::Fact> conditions, std::string forbiddenObject){

    std::string res = "NULL";
    double bestCost = -1.0;
    //we look for all possible refinment for the object
    std::vector<toaster_msgs::Fact> toCheck;
    for(std::vector<std::string>::iterator it = connector_->highLevelRefinment_[object].begin(); it != connector_->highLevelRefinment_[object].end(); it++){
        //we check the condition for the object
        if(*it == forbiddenObject){
            continue;
        }
        for(std::vector<toaster_msgs::Fact>::iterator itc = conditions.begin(); itc != conditions.end(); itc++){
            //we replace OBJECT by the object name
            toaster_msgs::Fact fact = *itc;
            if(itc->subjectId == "OBJECT"){
                fact.subjectId = *it;
            }
            if(itc->targetId == "OBJECT"){
                fact.targetId = *it;
            }
            toCheck.push_back(fact);
        }
    }
    //we only call the database once for all objects
    std::vector<std::string> inDB = AreFactsInDB(toCheck);
    int c = 0;
    for(std::vector<std::string>::iterator it = connector_->highLevelRefinment_[object].begin(); it != connector_->highLevelRefinment_[object].end(); it++){
        //if the preconditions are checked for this object
        if(*it == forbiddenObject){
            continue;
        }
        bool ok = true;
        for(int i = c; i < c + conditions.size(); i++){
            if(inDB[i] == "false"){
                ok = false;
                break;
            }
        }
        c = c + conditions.size();
        if(ok){
            //we look for the cost of this object
            if(connector_->humanDistances_[*it] > bestCost){
                res = *it;
                bestCost = connector_->humanDistances_[*it];
            }
        }
    }

    return res;
}

/**
 * \brief Function which check if facts are in db individually
 * @param facts list of facts to check
 * @return vector of bool which indicates if each fact is in the db
 * */
std::vector<std::string> VirtualAction::AreFactsInDB(std::vector<toaster_msgs::Fact> facts){

    std::vector<std::string> res;
    toaster_msgs::ExecuteDB srv;
    srv.request.command = "ARE_IN_TABLE";
    srv.request.type = "INDIV";
    srv.request.agent = connector_->robotName_;
    srv.request.facts = facts;
    if (connector_->client_db_execute_.call(srv)){
        res = srv.response.results;
    }else{
       ROS_ERROR("[action_executor] Failed to call service database_manager/execute");
    }
    return res;
}

/**
 * \brief Function which save in a file a gtp traj
 * @param traj the traj to save
 * */
void VirtualAction::saveTraj(trajectory_msgs::JointTrajectory traj, std::string trajName){

    connector_->fileSave_ <<"      " << trajName << ":" << std::endl;

    connector_->fileSave_ <<"       " << " joint_names: [";
    bool first = true;
    for(int i = 0; i < traj.joint_names.size(); i++){
        if(!first){
            connector_->fileSave_ <<", ";
        }
        connector_->fileSave_ <<"'" << traj.joint_names[i] << "'";
        first = false;
    }
    connector_->fileSave_ <<"]" << std::endl;

    connector_->fileSave_ <<"       " << " nbPoints: " << traj.points.size() << std::endl;
    for(int i = 0; i < traj.points.size(); i++){
        connector_->fileSave_ <<"        " << "point_" << i << ":" << std::endl;
        connector_->fileSave_ <<"          "  << " positions: [";
        first = true;
        for(int j = 0; j < traj.points[i].positions.size(); j++){
            if(!first){
                connector_->fileSave_ <<", ";
            }
            connector_->fileSave_  << traj.points[i].positions[j];
            first = false;
        }
        connector_->fileSave_ <<"]" << std::endl;
        connector_->fileSave_ <<"          "  << " velocities: [";
        first = true;
        for(int j = 0; j < traj.points[i].velocities.size(); j++){
            if(!first){
                connector_->fileSave_ <<", ";
            }
            connector_->fileSave_ << traj.points[i].velocities[j];
            first = false;
        }
        connector_->fileSave_ <<"]" << std::endl;
        connector_->fileSave_  <<"          "  << " accelerations: [";
        first = true;
        for(int j = 0; j < traj.points[i].accelerations.size(); j++){
            if(!first){
                connector_->fileSave_ <<", ";
            }
            connector_->fileSave_ << traj.points[i].accelerations[j];
            first = false;
        }
        connector_->fileSave_ <<"]" << std::endl;
        connector_->fileSave_ <<"          "  << " effort: [";
        first = true;
        for(int j = 0; j < traj.points[i].effort.size(); j++){
            if(!first){
                connector_->fileSave_ <<", ";
            }
            connector_->fileSave_  << traj.points[i].effort[j];
            first = false;
        }
        connector_->fileSave_ <<"]" << std::endl;
    }

}
