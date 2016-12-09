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

    bool foundArm = false;
    bool foundPos = false;
    for(int i=0; i<=action.parameter_keys.size();i++){
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
        ROS_WARN("[action_executor] Missing parameter: arm to move");
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

   //Here we fill the needed information for gtp (fill free to use other planner if needed)
   std::vector<gtp_ros_msgs::ActionId> attachments;
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

   //do not forget to replace action_name in the following line
   std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP("moveTo", agents, objects, datas, points, attachments);
   gtpActionId_ = answer.first;

   if(gtpActionId_ == -1){
       return false;
    }else{
       subSolutions_ = answer.second;
       return true;
    }

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

    int armId;
    if(arm_ == "right"){
        armId = 0;
    }else{
        armId = 1;
    }

    return executeTrajectory(gtpActionId_, 0, armId, action_server);

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
