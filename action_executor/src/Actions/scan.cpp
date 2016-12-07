/**
author Sandra Devin
Class allowing the execution of a scan action
**/

#include "action_executor/Actions/scan.h"

/**
 * \brief Constructor of the class
 * @param action the definition of the action to execute
 * @param connector pointer to the connector structure
 * */
Scan::Scan(supervisor_msgs::Action action, Connector* connector) : VirtualAction(connector){

    bool found = false;
    for(int i=0; i<=action.parameter_keys.size();i++){
        if(action.parameter_keys[i] == "object"){
            object_ = action.parameter_values[i];
            found = true;
            break;
        }
    }
    if(!found){
        ROS_WARN("[action_executor] Missing parameter: object to scan");
    }

    connector_->node_->getParam("/action_executor/timeScan", timeScan_);

    client_light_ = connector_->node_->serviceClient<dynamic_reconfigure::Reconfigure>("/camera_synchronizer_node/set_parameters");
}

/**
 * \brief Precondition of the scan action:
 *    - the object should be reachable by the agent
 * @return true if the preconditions are checked
 * */
bool Scan::preconditions(){

    //We check that the object is reachable by the agent
    std::vector<toaster_msgs::Fact> precsTocheck;
    toaster_msgs::Fact fact;
    fact.subjectId = object_;
    fact.property = "isReachableBy";
    fact.targetId = connector_->robotName_;
    precsTocheck.push_back(fact);

    return ArePreconditionsChecked(precsTocheck);

}

/**
 * \brief Planning the scan action:
 *    - no planning needed
 * @return true
 * */
bool Scan::plan(){

    return true;
}

/**
 * \brief Execution of the scan action:
 *    - go to rest pose if needed
 *    - turn on the light and wait
 * @return true if the execution succeed
 * */
bool Scan::exec(Server* action_server){

    //We go to rest pose if needed
    if(connector_->rightArmPose_ != connector_->rightArmRestPose_){
        supervisor_msgs::Action mvToAction;
        mvToAction.name = "moveTo";
        mvToAction.actors.push_back(connector_->robotName_);
        mvToAction.parameter_keys.push_back("position");
        mvToAction.parameter_values.push_back(connector_->rightArmRestPose_);
        mvToAction.parameter_keys.push_back("arm");
        mvToAction.parameter_values.push_back("right");

        MoveTo mvToServer(mvToAction, connector_);
        if(mvToServer.preconditions() && mvToServer.plan() && mvToServer.exec(action_server)){
           mvToServer.post();
        }else{
           ROS_WARN("[action_executor] Impossible to retract right arm");
        }
    }
    if(connector_->leftArmPose_ != connector_->leftArmRestPose_){
        supervisor_msgs::Action mvToAction;
        mvToAction.name = "moveTo";
        mvToAction.actors.push_back(connector_->robotName_);
        mvToAction.parameter_keys.push_back("position");
        mvToAction.parameter_values.push_back(connector_->leftArmRestPose_);
        mvToAction.parameter_keys.push_back("arm");
        mvToAction.parameter_values.push_back("left");

        MoveTo mvToServer(mvToAction, connector_);
        if(mvToServer.preconditions() && mvToServer.plan() && mvToServer.exec(action_server)){
           mvToServer.post();
        }else{
           ROS_WARN("[action_executor] Impossible to retract right arm");
        }
    }

    //TODO: add verification of head focus

    if(!connector_->simu_){
        //turn on the light
    }

    ros::Duration(timeScan_).sleep();

    if(!connector_->simu_){
        //turn off the light
    }

   return true;

}

/**
 * \brief Post conditions of the template action:
 *    - describe here the post conditions
 * @return true if the post-conditions are ok
 * */
bool Scan::post(){

    //Here we check/apply post conditions

    return true;
}

/**
 * \brief Control the red light in the robot head
 * @param on true if switch on requested, false if switch off
 * */
void Scan::controlRobotLight(bool on){

    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    int_param.name = "projector_mode";
    if(on){
        int_param.value = 3;
    }else{
        int_param.value = 1;
    }
    conf.ints.push_back(int_param);
    srv.request.config = conf;

    if (!client_light_.call(srv)){
     ROS_ERROR("Failed to call service camera_synchronizer_node/set_parameters");
    }
}
