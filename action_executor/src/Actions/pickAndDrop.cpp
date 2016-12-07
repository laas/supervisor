/**
author Sandra Devin
Class allowing the execution of a pick and drop action
**/

#include "action_executor/Actions/pickAndDrop.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
PickAndDrop::PickAndDrop(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    //we construct the pick action, then the drop action
    Pick* pick = new Pick(action, connector);
    pickAction_ = *pick;
    Drop* drop = new Drop(action, connector);
    dropAction_ = *drop;

    bool foundObj = false;
    bool foundCont = false;
    for(int i=0; i<=action.parameter_keys.size();i++){
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
}

/**
 * \brief Precondition of the pick and drop action:
 *    - the object should be a manipulable object
 *    - the container should be a container object
 *    - the object should be reachable by the agent
 *    - the container should be reachable by the agent
 * @return true if the preconditions are checked
 * */
bool PickAndDrop::preconditions(){

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

    //Then we check if object and the container are reachable
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = container_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the pick and drop action:
 *    - plan for pick action then for drop
 * @return true if the planning succeed
 * */
bool PickAndDrop::plan(){

    if(pickAction_.plan()){
        return dropAction_.plan();
    }

    return false;
}

/**
 * \brief Execution of the pick and drop place action:
 *    - execute the pick action then the drop
 * @return true if the execution succeed
 * */
bool PickAndDrop::exec(Server* action_server){

    if(pickAction_.exec(action_server) && pickAction_.post()){
        return dropAction_.exec(action_server);
    }

    return false;

}

/**
 * \brief Post conditions of the pick and drop action:
 *    - post condition of the drop action
 * @return true if the post-conditions are ok
 * */
bool PickAndDrop::post(){

    return dropAction_.post();
}
