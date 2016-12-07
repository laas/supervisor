/**
author Sandra Devin
Class allowing the execution of a place reachable action
**/

#include "action_executor/Actions/placeReachable.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
PlaceReachable::PlaceReachable(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    bool foundObj = false;
    bool foundSup = false;
    bool foundAg = false;
    for(int i=0; i<=action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "object"){
            object_ = action.parameter_values[i];
            foundObj = true;
        }
        if(action.parameter_keys[i] == "support"){
            support_ = action.parameter_values[i];
            foundSup = true;
        }
        if(action.parameter_keys[i] == "targetAgent"){
            targetAgent_ = action.parameter_values[i];
            foundAg = true;
        }
        if(foundObj && foundSup && foundAg){
            break;
        }
    }
    if(!foundObj){
        ROS_WARN("[action_executor] Missing parameter: object to place");
    }
    if(!foundSup){
        ROS_WARN("[action_executor] Missing parameter: support where to place");
    }
    if(!foundAg){
        ROS_WARN("[action_executor] Missing parameter: target agent");
    }


    replacementSupport_ = "NONE";
}

/**
 * \brief Precondition of the place action:
 *    - check if we have a previous grasp for the object
 *    - the object should be a manipulable object
 *    - the support should be a support object
 *    - the support should be reachable by the agent
 *    - the agent should hold the object
 * @return true if the preconditions are checked
 * */
bool PlaceReachable::preconditions(){

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

    //Then we check if the robot has the object in hand and if the support is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = support_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the place action:
 *    - check if we have a previous grasp for the object
 *    - check if other placement to use for planning
 *    - ask a plan to gtp
 * @return true if the planning succeed
 * */
bool PlaceReachable::plan(){

    //If there is no previous task we look for a previous grasp
    if(connector_->previousId_ == -1){
        if(connector_->idGrasp_ == -1){
            ROS_WARN("[action_executor] No previous Id nore previous grasp id");
           return false;
        }else{
            //TODO: add attachment in GTP
        }
    }

    std::vector<gtp_ros_msgs::Role> agents;
    gtp_ros_msgs::Role role;
    role.role = "mainAgent";
    role.name = connector_->robotName_;
    agents.push_back(role);
    role.role = "targetAgent_";
    role.name = targetAgent_;
    agents.push_back(role);
    std::vector<gtp_ros_msgs::Role> objects;
    role.role = "mainObject";
    role.name = object_;
    objects.push_back(role);
    std::vector<gtp_ros_msgs::Point> points;
    std::vector<gtp_ros_msgs::MiscData> datas;

    if(connector_->shouldUseRightHand_){
        gtp_ros_msgs::MiscData data;
        data.key = "hand";
        data.value = "right";
        datas.push_back(data);
    }

    std::string replacementTopic = "/action_executor/planningSupport/" + support_;
    //we look if there is a more specific placement to use for planning
    if(connector_->node_->hasParam(replacementTopic)){
        connector_->node_->getParam(replacementTopic, replacementSupport_);
        role.role = "supportObject";
        role.name = replacementSupport_;
        objects.push_back(role);
    }else{
        role.role = "supportObject";
        role.name = support_;
        objects.push_back(role);
    }

    std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP("placereachable", agents, objects, datas, points);
    gtpActionId_ = answer.first;

    if(gtpActionId_ == -1){
        return false;
     }

    subSolutions_ = answer.second;
    return true;
}

/**
 * \brief Execution of the place action:
 *    - execute the gtp plan
 * @return true if the execution succeed
 * */
bool PlaceReachable::exec(Server* action_server){

   return execAction(gtpActionId_, subSolutions_, false, action_server);

}

/**
 * \brief Post conditions of the place action:
 *    - put the object on the support
 * @return true if the post-conditions are ok
 * */
bool PlaceReachable::post(){

    if(replacementSupport_ != "NONE"){
        PutOnSupport(object_, replacementSupport_);
    }
    else{
        PutOnSupport(object_, support_);
    }

    return true;
}
