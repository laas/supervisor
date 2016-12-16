/**
author Sandra Devin
**/

#include "action_executor/action_executor.h"

/**
 * \brief Construction of the class
 * @param name name of the action server
 * @param node pointer to the node handle
 * */
ActionExecutor::ActionExecutor(std::string name, ros::NodeHandle* node):
action_server_(*node, name,
    boost::bind(&ActionExecutor::execute,this, _1), false)
 {
    action_server_.start();
    connector_.node_ = node;
    connector_.previousId_ = -1;
    connector_.idGrasp_ = -1;
    connector_.gripperRightOpen_ = false;
    connector_.gripperLeftOpen_ = false;
    connector_.rightArmMoving_ = false;
    connector_.leftArmMoving_ = false;
    connector_.rightGripperMoving_ = false;
    connector_.leftGripperMoving_ = false;
    connector_.torsoMoving_ = false;
    connector_.stopOrder_ = false;
    connector_.refineOrder_ = false;
    connector_.rightArmPose_ = "unknown";
    connector_.leftArmPose_ = "unknown";
    connector_.node_->getParam("/action_executor/restPosition/right", connector_.rightArmRestPose_);
    connector_.node_->getParam("/action_executor/restPosition/left", connector_.leftArmRestPose_);
    connector_.node_->getParam("/supervisor/waitActionServer", connector_.waitActionServer_);
    connector_.node_->getParam("/action_executor/shouldUseRightHand", connector_.shouldUseRightHand_);
    connector_.node_->getParam("/supervisor/simu", connector_.simu_);
    connector_.node_->getParam("/supervisor/robot/name", connector_.robotName_);
    connector_.node_->getParam("/action_executor/nbPlanMaxGTP", connector_.nbPlanMax_);
    connector_.node_->getParam("/supervisor/robot/toasterName", connector_.robotToaster_);
    connector_.node_->getParam("/action_executor/noExec", connector_.noExec_);
    connector_.node_->getParam("/action_executor/humanCost", connector_.humanCost_);

    initHighLevelNames();

    //Init services
    connector_.client_db_execute_ = connector_.node_->serviceClient<toaster_msgs::ExecuteDB>("database_manager/execute");
    connector_.client_put_hand_ = connector_.node_->serviceClient<toaster_msgs::PutInHand>("pdg/put_in_hand");
    connector_.client_remove_hand_ = connector_.node_->serviceClient<toaster_msgs::RemoveFromHand>("pdg/remove_from_hand");
    connector_.client_set_pose_ = connector_.node_->serviceClient<toaster_msgs::SetEntityPose>("pdg/set_entity_pose");
    connector_.client_gtp_traj_ = connector_.node_->serviceClient<gtp_ros_msgs::PublishTraj>("gtp/publishTraj");


    //Init action clients
    ROS_INFO("[action_executor] Waiting for gtp actions server.");
    connector_.acGTP_ = new actionlib::SimpleActionClient<gtp_ros_msgs::PlanAction>("gtp_server", true);
    connector_.acGTP_->waitForServer();
    ROS_INFO("[action_executor] Waiting for pr2motion actions server.");
    connector_.PR2motion_init_ = new actionlib::SimpleActionClient<pr2motion::InitAction>("pr2motion/Init", true);
    connector_.PR2motion_init_->waitForServer();
    connector_.PR2motion_torso_ = new actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>("pr2motion/Torso_Move", true);
    connector_.PR2motion_torso_->waitForServer();
    connector_.PR2motion_arm_right_ = new actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>("pr2motion/Arm_Right_Move",true);
    connector_.PR2motion_arm_right_->waitForServer();
    connector_.PR2motion_arm_left_ = new actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>("pr2motion/Arm_Left_Move",true);
    connector_.PR2motion_arm_left_->waitForServer();
    connector_.PR2motion_gripper_right_ = new actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>("pr2motion/Gripper_Right_Operate",true);
    connector_.PR2motion_gripper_right_->waitForServer();
    connector_.PR2motion_gripper_left_ = new actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>("pr2motion/Gripper_Left_Operate",true);
    connector_.PR2motion_gripper_left_->waitForServer();
    ROS_INFO("[action_executor] Action clients started.");

    //Init PR2motion
    ros::ServiceClient connect = connector_.node_->serviceClient<pr2motion::connect_port>("pr2motion/connect_port");
    pr2motion::InitGoal goal_init;
    connector_.PR2motion_init_->sendGoal(goal_init);

    pr2motion::connect_port srv;
    srv.request.local = "joint_state";
    srv.request.remote = "joint_states";
    if (!connect.call(srv)){
      ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "head_controller_state";
    srv.request.remote = "/head_traj_controller/state";
    if (!connect.call(srv)){
       ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
    }
    srv.request.local = "traj";
    srv.request.remote = "gtp_trajectory";
    if (!connect.call(srv)){
       ROS_ERROR("[action_executor] Failed to call service pr2motion/connect_port");
    }

    if(connector_.simu_){//change torso position
      pr2motion::Torso_MoveGoal goal;
      goal.torso_position = 0.1;
      connector_.torsoMoving_ = true;
      connector_.PR2motion_torso_->sendGoal(goal);
      ROS_INFO("[action_executor] Waiting for Torso move");
      bool finishedBeforeTimeout = connector_.PR2motion_torso_->waitForResult(ros::Duration(connector_.waitActionServer_));
      connector_.torsoMoving_ = false;
      if (!finishedBeforeTimeout){
         ROS_INFO("[action_executor] Action PR2Torso did not finish before the time out.");
      }
    }

    ROS_INFO("[action_executor] Action server ready");
}

/**
 * \brief Function to execute an action
 * @param goal the description of the action to execute
 * */
void ActionExecutor::execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal) {

    connector_.timerStart_ = ros::Time::now();
    ros::Time now;

    //Checking the action parameters are valids
    if(goal->action.parameter_keys.size() != goal->action.parameter_values.size()){
        ROS_ERROR("[action_executor] In valid parameters: nb keys should be equal to nb values!");
        result_.report = false;
        result_.state = "NON_VALID";
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        action_server_.setAborted(result_);
        return;
    }

    //Getting action informations
    std::string name = goal->action.name;
    int id = goal->action.id;
    ROS_INFO("Executing action %s with id %i with parameters:", name.c_str(), id);
    for(int i=0; i<goal->action.parameter_keys.size();i++){
        ROS_INFO("  %s: %s", goal->action.parameter_keys[i].c_str(), goal->action.parameter_values[i].c_str());
    }

    //Creating the action
    VirtualAction* act = NULL;
    act = initializeAction(goal->action);
    if(!act){
        result_.report = false;
        result_.state = "NON_VALID";
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        action_server_.setAborted(result_);
        ROS_INFO("[action_executor] Action failed at creation");
        return;
    }

    connector_.isActing_ = true;
    connector_.currentAction_ = goal->action;

    //Checking preconditions
    feedback_.state = "PREC";
    action_server_.publishFeedback(feedback_);
    if(!act->preconditions()){
        connector_.isActing_ = false;
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
            ROS_INFO("[action_executor] Action failed in preconditions");
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);
            ROS_INFO("[action_executor] Action stoped");
        }
        return;
    }

    if(action_server_.isPreemptRequested() || connector_.stopOrder_){
        connector_.isActing_ = false;
        result_.state = "PREEMPTED";
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        action_server_.setPreempted(result_);
        ROS_INFO("[action_executor] Action stoped");
        return;
    }

    //Plan for the action
    feedback_.state = "PLAN";
    action_server_.publishFeedback(feedback_);
    if(!act->plan()){
        connector_.isActing_ = false;
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
            ROS_INFO("[action_executor] Action failed in planning");
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);
            ROS_INFO("[action_executor] Action stoped");
        }
        return;
    }

    if(action_server_.isPreemptRequested() || connector_.stopOrder_){
        connector_.isActing_ = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        result_.state = "PREEMPTED";
        result_.report = false;
        result_.shouldRetractRight = false;
        result_.shouldRetractLeft = false;
        action_server_.setPreempted(result_);
        ROS_INFO("[action_executor] Action stoped");
        return;
    }

    //Execution of the action
    feedback_.state = "EXEC";
    action_server_.publishFeedback(feedback_);
    if(!act->exec(&action_server_)){
        connector_.isActing_ = false;
        result_.report = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        if(connector_.rightArmRestPose_ != connector_.rightArmPose_){
            result_.shouldRetractRight = true;
        }else{
            result_.shouldRetractRight = false;
        }
        if(connector_.leftArmRestPose_ != connector_.leftArmPose_){
            result_.shouldRetractLeft = true;
        }else{
            result_.shouldRetractLeft = false;
        }
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
            ROS_INFO("[action_executor] Action failed in execution");
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);
            ROS_INFO("[action_executor] Action stoped");
        }
        return;
    }

    //Apply/Check Post-conditions
    feedback_.state = "POST";
    action_server_.publishFeedback(feedback_);
    if(!act->post()){
        connector_.isActing_ = false;
        result_.report = false;
        now = ros::Time::now();
        ros::Duration d = now - connector_.timerStart_;
        result_.timeTot = d.toSec();
        result_.timeDB = connector_.timeDB_;
        result_.timePlan = connector_.timePlan_;
        result_.timeExec = connector_.timeExec_;
        result_.timeGTP = connector_.timeGTP_;
        if(connector_.rightArmRestPose_ != connector_.rightArmPose_){
            result_.shouldRetractRight = true;
        }else{
            result_.shouldRetractRight = false;
        }
        if(connector_.leftArmRestPose_ != connector_.leftArmPose_){
            result_.shouldRetractLeft = true;
        }else{
            result_.shouldRetractLeft = false;
        }
        if(!action_server_.isPreemptRequested()){
            result_.state = feedback_.state;
            action_server_.setAborted(result_);
            ROS_INFO("[action_executor] Action failed in post conditions");
        }else{
            result_.state = "PREEMPTED";
            action_server_.setPreempted(result_);
            ROS_INFO("[action_executor] Action stoped");
        }
        return;
    }

    now = ros::Time::now();
    ros::Duration d = now - connector_.timerStart_;
    result_.timeTot = d.toSec();
    result_.timeDB = connector_.timeDB_;
    result_.timePlan = connector_.timePlan_;
    result_.timeExec = connector_.timeExec_;
    result_.timeGTP = connector_.timeGTP_;
    if(connector_.rightArmRestPose_ != connector_.rightArmPose_){
        result_.shouldRetractRight = true;
    }else{
        result_.shouldRetractRight = false;
    }
    if(connector_.leftArmRestPose_ != connector_.leftArmPose_){
        result_.shouldRetractLeft = true;
    }else{
        result_.shouldRetractLeft = false;
    }
    connector_.isActing_ = false;
    result_.report = true;
    result_.state = "OK";
    action_server_.setSucceeded(result_);
    ROS_INFO("[action_executor] Action succeed");
}


/**
 * \brief Initialize a virtual action based on its name and params
 * @param action the description of the action to initialize
 * @return a virtual action initialized
 * */
VirtualAction* ActionExecutor::initializeAction(supervisor_msgs::Action action) {
    VirtualAction* act = NULL;

    connector_.stopOrder_ = false;

    if(action.name == "pick"){
        act = new Pick(action, &connector_);
    }else if(action.name == "place"){
        act = new Place(action, &connector_);
    }else if(action.name == "placeReachable"){
        act = new PlaceReachable(action, &connector_);
    }else if(action.name == "drop"){
        act = new Drop(action, &connector_);
    }else if(action.name == "scan"){
        act = new Scan(action, &connector_);
    }else if(action.name == "moveTo"){
        act = new MoveTo(action, &connector_);
    }else if(action.name == "pickAndPlace"){
        act = new PickAndPlace(action, &connector_);
    }else if(action.name == "pickAndPlaceReachable"){
        act = new PickAndPlaceReachable(action, &connector_);
    }else if(action.name == "pickAndDrop"){
        act = new PickAndDrop(action, &connector_);
    }else{
        ROS_WARN("[action_executor] Unknown action");
    }

    return act;
}

/**
 * \brief Initialize the high level name maps based on params
 * */
void ActionExecutor::initHighLevelNames() {

    std::vector<std::string> manipulableObjects;
    connector_.node_->getParam("/entities/objects", manipulableObjects);
    for(std::vector<std::string>::iterator it = manipulableObjects.begin(); it != manipulableObjects.end(); it++){
        std::string paramName = "/entities/highLevelName/" + *it;
        if(connector_.node_->hasParam(paramName)){
            connector_.node_->getParam(paramName, connector_.highLevelRefinment_[*it]);
            for(std::vector<std::string>::iterator ith = connector_.highLevelRefinment_[*it].begin(); ith != connector_.highLevelRefinment_[*it].end(); ith++){
                connector_.highLevelNames_[*it] = *ith;
            }
        }
    }
}
