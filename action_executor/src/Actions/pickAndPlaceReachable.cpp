/**
author Sandra Devin
Class allowing the execution of a pick and place reachable action
**/

#include "action_executor/Actions/pickAndPlaceReachable.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
PickAndPlaceReachable::PickAndPlaceReachable(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    //we construct the pick action, then the place action
    Pick* pick = new Pick(action, connector);
    pickAction_ = *pick;
    PlaceReachable* place = new PlaceReachable(action, connector);
    placeAction_ = *place;

    bool foundObj = false;
    bool foundSup = false;
    for(int i=0; i<=action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "object"){
            object_ = action.parameter_values[i];
            foundObj = true;
        }
        if(action.parameter_keys[i] == "support"){
            support_ = action.parameter_values[i];
            foundSup = true;
        }
        if(foundObj && foundSup){
            break;
        }
    }
    if(!foundObj){
        ROS_WARN("[action_executor] Missing parameter: object to place");
    }
    if(!foundSup){
        ROS_WARN("[action_executor] Missing parameter: support where to place");
    }
}

/**
 * \brief Precondition of the pick and place reachable action:
 *    - the object should be a manipulable object
 *    - the support should be a support object
 *    - the object should be reachable by the agent
 *    - the support should be reachable by the agent
 * @return true if the preconditions are checked
 * */
bool PickAndPlaceReachable::preconditions(){

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
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);
    fact.subjectId = support_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the pick and place reachable action:
 *    - plan for pick action then for place
 * @return true if the planning succeed
 * */
bool PickAndPlaceReachable::plan(){

    if(pickAction_.plan()){
        return placeAction_.plan();
    }

    return false;
}

/**
 * \brief Execution of the pick and place place reachablevaction:
 *    - execute the pick action then the place
 * @return true if the execution succeed
 * */
bool PickAndPlaceReachable::exec(Server* action_server){

    if(pickAction_.exec(action_server) && pickAction_.post()){
        return placeAction_.exec(action_server);
    }

    return false;

}

/**
 * \brief Post conditions of the pick and place reachable action:
 *    - post condition of the place action
 * @return true if the post-conditions are ok
 * */
bool PickAndPlaceReachable::post(){

    return placeAction_.post();
}
