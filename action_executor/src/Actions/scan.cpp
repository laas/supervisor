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

    //we look for the action parameters
    bool found = false;
    for(int i=0; i<action.parameter_keys.size();i++){
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
    connector_->node_->getParam("/action_executor/timeWaitScan", timeWaitScan_);

    client_light_ = connector_->node_->serviceClient<dynamic_reconfigure::Reconfigure>("/camera_synchronizer_node/set_parameters");
    sub_head_focus_ = connector_->node_->subscribe("simple_head_manager/focus", 1, &Scan::focusCallback, this);
}

/**
 * \brief Callback of the head focus topic
 * @param msg topic msg
 * */
void Scan::focusCallback(const std_msgs::String::ConstPtr& msg){

    headFocus_ = msg->data;
}

/**
 * \brief Precondition of the scan action:
 *    - look for an object refinment if needed
 *    - the object should be reachable by the agent
 * @return true if the preconditions are checked
 * */
bool Scan::preconditions(){

    if(!isRefined(object_)){
        //if the object is not refined, we look fo a refinement
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
    connector_->currentAction_.shouldKeepFocus = true;

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
    /*if(connector_->leftArmPose_ != connector_->leftArmRestPose_){
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
    }*/

    //Verification of head focus
    bool shouldWait = true;
    ros::Time start_ = ros::Time::now();
    while(shouldWait){
        ros::Duration d = ros::Time::now() - start_;
        double duration = d.toSec();
        if(duration > timeWaitScan_){
            shouldWait = false;
        }
        if(headFocus_ == object_){
            shouldWait = false;
        }
    }

    if(headFocus_ != object_){
        ROS_WARN("Unable to get the head focus, aborting the action!");
        return false;
    }

    if(!connector_->simu_){
        //turn on the light
        controlRobotLight(true);
    }

    ROS_INFO("SCANNING");
    ros::Duration(timeScan_).sleep();

    if(!connector_->simu_){
        //turn off the light
        controlRobotLight(false);
    }

   return true;

}

/**
 * \brief Post conditions of the template action:
 *    - describe here the post conditions
 * @return true if the post-conditions are ok
 * */
bool Scan::post(){

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
