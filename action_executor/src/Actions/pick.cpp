/**
author Sandra Devin
Class allowing the execution of a pick action
**/

#include "action_executor/Actions/pick.h"

/**
 * \brief Construction of the class
 * @param action the definition of the pick action to execute
 * @param connector pointer to the connector structure
 * */
Pick::Pick(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){
    bool found = false;
    for(int i=0; i<action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "object"){
            object_ = action.parameter_values[i];
            found = true;
            break;
        }
    }
    if(!found){
        ROS_WARN("[action_executor] Missing parameter: object to pick");
    }
    actionName_ = "pick";
    inRefinement_ = false;
}

/**
 * \brief Precondition of the pick action:
 *    - look for an object refinment if needed
 *    - the object should be a manipulable object
 *    - the object should be reachable by the agent
 *    - the agent should not have any object in hand
 * @return true if the preconditions are checked
 * */
bool Pick::preconditions(){

    //if the object is not refined, we look fo a refinement
    if(!isRefined(object_)){
        //we look for a refinment
        std::vector<toaster_msgs::Fact> conditions;
        toaster_msgs::Fact fact;
        fact.subjectId = "NULL";
        fact.property = "isOn";
        fact.targetId = "OBJECT";
        conditions.push_back(fact);
        fact.subjectId = "OBJECT";
        fact.property = "isReachableBy";
        fact.targetId = connector_->robotName_;
        conditions.push_back(fact);
        std::string newObject  = findRefinment(object_, conditions, "NULL");
        //we update the current action
        if(newObject != "NULL"){
            initialObject_ = object_;
            std::replace (connector_->currentAction_.parameter_values.begin(), connector_->currentAction_.parameter_values.end(), object_, newObject);
            object_ = newObject;
        }else{
            ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
            return false;
        }
    }

    //the robot should look at the object
    connector_->currentAction_.headFocus = object_;
    connector_->currentAction_.shouldKeepFocus = false;

    //First we check if the object is a known manipulable object
    if(!isManipulableObject(object_)){
      ROS_WARN("[action_executor] The object to pick (%s) is not a known manipulable object", object_.c_str());
      return false;
    }

    //Then we check if the robot has the hands free and if the object is reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = "NULL";
    fact.property = "isHoldBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = object_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the pick action:
 *    - ask a gtp plan
 * @return true if the planning succeed
 * */
bool Pick::plan(){

    //FOR TESTS ONLY
    connector_->previousId_  = -1;

    if(connector_->saveMode_ == "load"){
        gtpActionId_ = 100;
        gtp_ros_msgs::SubSolution subSol;
        subSol.agent = "PR2_ROBOT";
        if(param2_ == "place"){
            subSol.armId = 0;
        }else{
            subSol.armId = 1;
        }
        subSol.id = 0;
        subSol.name = "approach";
        subSol.type = "move";
        subSolutions_.push_back(subSol);
        subSol.id = 1;
        subSol.name = "engage";
        subSol.type = "move";
        subSolutions_.push_back(subSol);
        subSol.id = 2;
        subSol.name = "grasp";
        subSol.type = "grasp";
        subSolutions_.push_back(subSol);
        subSol.id = 3;
        subSol.name = "escape";
        subSol.type = "move";
        subSolutions_.push_back(subSol);
    }else{
        //we ask gtp a plan
        std::vector<gtp_ros_msgs::ActionId> attachments;
        std::vector<gtp_ros_msgs::Role> agents;
        gtp_ros_msgs::Role role;
        role.role = "mainAgent";
        role.name = connector_->robotName_;
        agents.push_back(role);
        std::vector<gtp_ros_msgs::Role> objects;
        role.role = "mainObject";
        role.name = object_;
        objects.push_back(role);
        std::vector<gtp_ros_msgs::Point> points;
        std::vector<gtp_ros_msgs::MiscData> datas;

        if(param2_ == "place"){
            gtp_ros_msgs::MiscData data;
            data.key = "hand";
            data.value = "right";
            datas.push_back(data);
        }else{
            gtp_ros_msgs::MiscData data;
            data.key = "hand";
            data.value = "left";
            datas.push_back(data);
        }
        /*if(connector_->shouldUseRightHand_){
        gtp_ros_msgs::MiscData data;
        data.key = "hand";
        data.value = "right";
        datas.push_back(data);
        }*/

        std::pair<int, std::vector<gtp_ros_msgs::SubSolution> > answer = planGTP("pick", agents, objects, datas, points, attachments);
        gtpActionId_ = answer.first;

        if(gtpActionId_ == -1){
        return false;
        }

        subSolutions_ = answer.second;
    }



    return true;
}

/**
 * \brief Execution of the pick action:
 *    - execute gtp plan
 * @return true if the execution succeed
 * */
bool Pick::exec(Server* action_server){

   while(true){
       if(execAction(gtpActionId_, subSolutions_, true, action_server)){
           return true;
       }else if(connector_->refineOrder_ ){
	   connector_->refineOrder_  = false;
           connector_->previousId_  = -1;
           //the chosen object is already taken, we look for another refinement
           std::vector<toaster_msgs::Fact> conditions;
           toaster_msgs::Fact fact;
           fact.subjectId = "NULL";
           fact.property = "isOn";
           fact.targetId = "OBJECT";
           conditions.push_back(fact);
           fact.subjectId = "OBJECT";
           fact.property = "isReachableBy";
           fact.targetId = connector_->robotName_;
           conditions.push_back(fact);
           std::string newObject  = findRefinment(initialObject_, conditions, object_);
           //we update the current action
           if(newObject != "NULL"){
               std::replace (connector_->currentAction_.parameter_values.begin(), connector_->currentAction_.parameter_values.end(), object_, newObject);
               object_ = newObject;
               connector_->currentAction_.headFocus = object_;
               if(!this->plan()){
                   return false;
               }
           }else{
               ROS_WARN("[action_executor] No possible refinement for object: %s", object_.c_str());
               return false;
           }
       }else{
           connector_->previousId_  = -1;
           return false;
       }
   }

}

/**
 * \brief Post conditions of the pick action:
 *    - chek if the gripper is not empty
 * @return true if the post-conditions are ok
 * */
bool Pick::post(){

    if(object_ == connector_->onScanArea1_){
        connector_->onScanArea1_ = "NONE";
    }
    if(object_  == connector_->onScanArea2_){
        connector_->onScanArea2_ = "NONE";
    }

    std_msgs::String msg;
    msg.data = object_;
    connector_->pick_pub_.publish(msg);
    PutInHand(object_, "right", 0);

    //Check gripper position (completly close or not)
    if(gripperEmpty_  && !connector_->simu_){
        ROS_WARN("[action_executor] Robot failed to pick (gripper empty)");
        connector_->previousId_  = -1;
        return false;
    }

    return true;
}
