/**
author Sandra Devin
Class allowing the execution of a move to position action
**/

#include "action_executor/Actions/moveTo.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
MoveTo::MoveTo(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    //looking for the parameters of the action
    bool foundArm = false;
    bool foundPos = false;
    for(int i=0; i<action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "arm"){
            arm_ = action.parameter_values[i];
            foundArm = true;
        }
        if(action.parameter_keys[i] == "position"){
            position_ = action.parameter_values[i];
            foundPos = true;
        }
        if(foundArm && foundPos){
            break;
        }
    }
    if(!foundArm){
        ROS_WARN("[action_executor] Missing parameter: arm to move, right by default");
        arm_ = "right";
    }
    if(!foundPos){
        ROS_WARN("[action_executor] Missing parameter: position to reach");
    }
}

/**
 * \brief Precondition of the moveTo action:
 *    - no preconditions to check
 * @return true
 * */
bool MoveTo::preconditions(){

    return true;

}

/**
 * \brief Planning the moveTo action:
 *    - ask a plan to GTP
 * @return true if the planning succeed
 * */
bool MoveTo::plan(){

    //Check if the position is already reached
    if(arm_ == "right" && connector_->rightArmPose_ == position_){
         return true;
    }else if(arm_ == "left" && connector_->leftArmPose_ == position_){
         return true;
    }

   //ask gtp a plan
   /*std::vector<gtp_ros_msgs::ActionId> attachments;
   std::vector<gtp_ros_msgs::Role> agents;
   gtp_ros_msgs::Role role;
   role.role = "mainAgent";
   role.name = connector_->robotName_;
   agents.push_back(role);
   std::vector<gtp_ros_msgs::Role> objects;
   std::vector<gtp_ros_msgs::Point> points;
   std::vector<gtp_ros_msgs::MiscData> datas;
   gtp_ros_msgs::MiscData data;
   data.key = "confName";
   data.value = position_;
   datas.push_back(data);

   std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP("moveTo", agents, objects, datas, points, attachments);
   gtpActionId_ = answer.first;

   if(gtpActionId_ == -1){
       return false;
    }else{
       subSolutions_ = answer.second;
       return true;
    }*/

    return true;
}

/**
 * \brief Execution of the moveTo action:
 *    - execute the trajectory returned by GTP
 * @return true if the execution succeed
 * */
bool MoveTo::exec(Server* action_server){

    //Check if the position is already reached
    if(arm_ == "right" && connector_->rightArmPose_ == position_){
         return true;
    }else if(arm_ == "left" && connector_->leftArmPose_ == position_){
         return true;
    }

    if(arm_ == "right"){
        pr2motion::Arm_Right_MoveToQGoalGoal goalQ;
       goalQ.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
       goalQ.shoulder_pan_joint = -1.952888;
       goalQ.shoulder_lift_joint = -0.095935;
       goalQ.upper_arm_roll_joint = -0.601572;
       goalQ.elbow_flex_joint = -1.600124;
       goalQ.forearm_roll_joint = 0.018247;
       goalQ.wrist_flex_joint = -0.432897;
       goalQ.wrist_roll_joint = -1.730082;\
       connector_->PR2motion_arm_right_Q_->sendGoal(goalQ);
       connector_->rightArmMoving_ = true;
       ROS_INFO("[action_manager] Waiting for arms move");
       bool finishedBeforeTimeout = connector_->PR2motion_arm_right_Q_->waitForResult(ros::Duration(connector_->waitActionServer_));
       connector_->rightArmMoving_ = false;
       if (!finishedBeforeTimeout){
          ROS_INFO("Action PR2 go to Q did not finish before the time out.");
       }
    }
   else if(arm_ == "left"){
        pr2motion::Arm_Left_MoveToQGoalGoal goalQL;
        goalQL.traj_mode.value=pr2motion::pr2motion_TRAJ_MODE::pr2motion_TRAJ_GATECH;
        goalQL.shoulder_pan_joint = 1.91155;
        goalQL.shoulder_lift_joint = -0.0984492;
        goalQL.upper_arm_roll_joint = 0.6;
        goalQL.elbow_flex_joint = -1.6534;
        goalQL.forearm_roll_joint = -0.02173;
        goalQL.wrist_flex_joint = -0.473717;
        goalQL.wrist_roll_joint = -1.76561;\
        connector_->PR2motion_arm_left_Q_->sendGoal(goalQL);
        connector_->leftArmMoving_ = true;
        ROS_INFO("[action_manager] Waiting for arms move");
        bool finishedBeforeTimeout = connector_->PR2motion_arm_left_Q_->waitForResult(ros::Duration(connector_->waitActionServer_));
        connector_->leftArmMoving_ = false;
        if (!finishedBeforeTimeout){
          ROS_INFO("Action PR2 go to Q did not finish before the time out.");
        }
   }

       return true;


   /* //convert the armId in pr2motion format
    int armId;
    if(arm_ == "right"){
        armId = 0;
    }else{
        armId = 1;
    }

    //for moveTo, the solution is only one trajectory
    return executeTrajectory(gtpActionId_, 0, armId, action_server, "move");*/

}

/**
 * \brief Post conditions of the moveTo action:
 *    - update arm position
 * @return true if the post-conditions are ok
 * */
bool MoveTo::post(){

    if(arm_ == "right"){
        connector_->rightArmPose_ = position_;
    }else{
        connector_->leftArmPose_ = position_;
    }

    return true;
}
