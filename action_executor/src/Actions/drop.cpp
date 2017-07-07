/**
author Sandra Devin
Class allowing the execution of a drop action
**/

#include "action_executor/Actions/drop.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
Drop::Drop(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    //looking for the parameters of the action
    bool foundObj = false;
    bool foundCont = false;
    for(int i=0; i<action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "object"){
            object_ = action.parameter_values[i];
            foundObj = true;
        }
        if(action.parameter_keys[i] == "container"){
            container_ = action.parameter_values[i];
            foundCont = true;
        }
        if(foundObj && foundCont){
            break;
        }
    }
    if(!foundObj){
        ROS_WARN("[action_executor] Missing parameter: object to drop");
    }
    if(!foundCont){
        ROS_WARN("[action_executor] Missing parameter: container where to drop");
    }

    inRefinement_ = false;
    actionName_ = "drop";
    param1_ = "SCAN_AREA2";
    param2_ = container_;

}

/**
 * \brief Precondition of the drop action:
 *    - look for a container refinment if needed
 *    - the object should be a manipulable object
 *    - the container should be a container object
 *    - the container should be reachable by the agent
 *    - the agent should hold the object
 * @return true if the preconditions are checked
 * */
bool Drop::preconditions(){

    //if the container is not refined, we look fo a refinement
    if(!isRefined(container_)){
        //we look for a refinment
        std::vector<toaster_msgs::Fact> conditions;
        toaster_msgs::Fact fact;
        fact.subjectId = "OBJECT";
        fact.property = "isReachableBy";
        fact.targetId = connector_->robotName_;
        conditions.push_back(fact);
        std::string newObject  = findRefinment(container_, conditions, "NULL");
        //we update the current action
        if(newObject != "NULL"){
            initialContainer_ = container_;
            std::replace (connector_->currentAction_.parameter_values.begin(), connector_->currentAction_.parameter_values.end(), container_, newObject);
            container_ = newObject;
        }else{
            ROS_WARN("[action_executor] No possible refinement for container: %s", container_.c_str());
            return false;
        }
    }

    //the robot should look the container
    connector_->currentAction_.headFocus = container_;
    connector_->currentAction_.shouldKeepFocus = false;

    //First we check if the object is a known manipulable object
    if(!isManipulableObject(object_)){
       ROS_WARN("[action_executor] The object to place is not a known manipulable object");
      return false;
    }

    //Then we check if the support is a known support object
    if(!isContainerObject(container_)){
       ROS_WARN("[action_executor] The container is not a known container object");
       return false;
     }

    //Then we check if the robot has the object in hand and if the container is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isHoldBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = container_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the template action:
 *    - check if we have a previous grasp for the object
 *    - ask a plan to gtp
 * @return true if the planning succeed
 * */
bool Drop::plan(){


    if(connector_->saveMode_ == "load"){
        gtpActionId_ = 101;
        gtp_ros_msgs::SubSolution subSol;
        subSol.agent = "PR2_ROBOT";
        subSol.armId = 1;
        subSol.id = 0;
        subSol.name = "approach";
        subSol.type = "move";
        subSolutions_.push_back(subSol);
        subSol.id = 1;
        subSol.name = "release";
        subSol.type = "release";
        subSolutions_.push_back(subSol);
    }else{
        std::vector<gtp_ros_msgs::ActionId> attachments;
        //If there is no previous task we look for a previous grasp
        /*if(connector_->previousId_ == -1){
            if(connector_->idGrasp_ == -1){
                ROS_WARN("[action_executor] No previous Id nore previous grasp id");
               return false;
            }else{
                //TODO: add attachment in GTP
            }
        }*/

        //We ask gtp a plan
        std::vector<gtp_ros_msgs::Role> agents;
        gtp_ros_msgs::Role role;
        role.role = "mainAgent";
        role.name = connector_->robotName_;
        agents.push_back(role);
        std::vector<gtp_ros_msgs::Role> objects;
        role.role = "mainObject";
        role.name = object_;
        objects.push_back(role);
        role.role = "supportObject";
        role.name = container_;
        objects.push_back(role);
        std::vector<gtp_ros_msgs::Point> points;
        std::vector<gtp_ros_msgs::MiscData> datas;

        //if(connector_->shouldUseRightHand_){
            gtp_ros_msgs::MiscData data;
            data.key = "hand";
            data.value = "left";
            datas.push_back(data);
        //}


        std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP("drop", agents, objects, datas, points, attachments);
        gtpActionId_ = answer.first;

        if(gtpActionId_ == -1){
            return false;
         }

        subSolutions_ = answer.second;
    }
    return true;
}

/**
 * \brief Execution of the template action:
 *    - execute the gtp plan
 * @return true if the execution succeed
 * */
bool Drop::exec(Server* action_server){

    while(true){
        if(execAction(gtpActionId_, subSolutions_, false, action_server)){
            return true;
        }else if(connector_->refineOrder_ ){
            //the chosen object is already taken, we look for another refinement
            connector_->previousId_  = -1;
            std::vector<toaster_msgs::Fact> conditions;
            toaster_msgs::Fact fact;
            fact.subjectId = "OBJECT";
            fact.property = "isReachableBy";
            fact.targetId = connector_->robotName_;
            conditions.push_back(fact);
            std::string newObject  = findRefinment(initialContainer_, conditions, container_);
            //we update the current action
            if(newObject != "NULL"){
                std::replace (connector_->currentAction_.parameter_values.begin(), connector_->currentAction_.parameter_values.end(), container_, newObject);
                container_ = newObject;
                connector_->currentAction_.headFocus = container_;
                if(!this->plan()){
                    return false;
                }
            }else{
                ROS_WARN("[action_executor] No possible refinement for container: %s", container_.c_str());
                return false;
            }
        }else{
            connector_->previousId_  = -1;
            return false;
        }
    }

}

/**
 * \brief Post conditions of the template action:
 *    - put the object in the container
 * @return true if the post-conditions are ok
 * */
bool Drop::post(){

    PutInContainer(object_, container_);

    return true;
}
