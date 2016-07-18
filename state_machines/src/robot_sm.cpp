/**
author Sandra Devin

State machine for the robot
**/

#include <state_machines/robot_sm.h>



RobotSM::RobotSM():
actionClient_("supervisor/action_executor", true)
 {
	node_.getParam("/robot/name", robotName_);
	actionClient_.waitForServer();
	isActing_ = false;
    shouldRetractRight_ = true;
    shouldRetractLeft_ = true;
    ROS_INFO("[state_machines] Waiting for head action server");
    head_action_client = new actionlib::SimpleActionClient<pr2motion::Head_Move_TargetAction>("pr2motion/Head_Move_Target",true);
    head_action_client->waitForServer();
    ROS_INFO("[state_machines] Human state machine ready");
}


/*
Called once when the goal of the action client completes
*/
void RobotSM::doneCb(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result){

		isActing_ = false;
        shouldRetractRight_ = result->shouldRetractRight;
        shouldRetractLeft_ = result->shouldRetractLeft;
}

/*
State where the robot is IDLE
*/
string RobotSM::idleState(){

	//We look if the robot has an action to do
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    supervisor_msgs::GetInfo srv;
    srv.request.info ="ACTIONS_TODO";
	srv.request.agent = robotName_;
	srv.request.actor = robotName_;
	if (client.call(srv)){
	 if(srv.response.state == "READY"){//the robot has an action to do, we send it to the action manager
		supervisor_msgs::ActionExecutorGoal goal;
  		goal.action = srv.response.action;
  		actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
		isActing_ = true;
		ROS_INFO("[state_machines] Robot goes to ACTING");
		return "ACTING";
	 }else if(srv.response.state == "NEEDED"){//the robot has an action but not possible yet, we go to the WAITING state
		ROS_INFO("[state_machines] Robot goes to WAITING");
		return "WAITING";
	 }
	}else{
     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
	}

	lookAtHuman();

    //if no more action to do we retract if needed
    if(shouldRetractRight_){
        //retract right arm if needed
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action.name = "moveTo";
        string rightArmRestPose_;
        node_.getParam("/restPosition/right", rightArmRestPose_);
        goal.action.parameters.push_back(rightArmRestPose_);
        goal.action.actors.push_back(robotName_);
        actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
        isActing_ = true;
        ROS_INFO("[state_machines] Robot goes to ACTING");
        return "ACTING";
    }
    if(shouldRetractLeft_){
        //retract left arm if needed
        supervisor_msgs::ActionExecutorGoal goal;
        goal.action.name = "moveTo";
        string leftArmRestPose_;
        node_.getParam("/restPosition/left", leftArmRestPose_);
        goal.action.parameters.push_back(leftArmRestPose_);
        goal.action.actors.push_back(robotName_);
        actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
        isActing_ = true;
        ROS_INFO("[state_machines] Robot goes to ACTING");
        return "ACTING";
    }


	return "IDLE";
}

/*
State where the robot is ACTING
*/
string RobotSM::actingState(){

    //get objects and weight from action_executor topic


	if(!isActing_){
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
	}

	return "ACTING";
}

/*
State where the robot is WAITING
*/
string RobotSM::waitingState(){

	//We look if the robot has an action to do
    ros::ServiceClient client = node_.serviceClient<supervisor_msgs::GetInfo>("mental_state/get_info");
    supervisor_msgs::GetInfo srv;
    srv.request.info ="ACTIONS_TODO";
	srv.request.agent = robotName_;
	srv.request.actor = robotName_;
	if (client.call(srv)){
	 if(srv.response.state == "READY"){//the robot has an action to do, we send it to the action manager
		supervisor_msgs::ActionExecutorGoal goal;
  		goal.action = srv.response.action;
  		actionClient_.sendGoal(goal,  boost::bind(&RobotSM::doneCb, this, _1, _2), Client::SimpleActiveCallback(),  Client::SimpleFeedbackCallback());
		isActing_ = true;
		ROS_INFO("[state_machines] Robot goes to ACTING");
		return "ACTING";
	 }else if(srv.response.state == "NEEDED"){//the robot is still not possible, we stay in the WAITING state
		return "WAITING";
	 }else{// the robot has no more action to do, we return to IDLE
		ROS_INFO("[state_machines] Robot goes to IDLE");
		return "IDLE";
	  }
	}else{
     ROS_ERROR("[state_machines] Failed to call service mental_state/get_info");
	}
	
	return "WAITING";
}

void RobotSM::lookAtHuman(){


    //we get the coordonates of the object
    toaster_msgs::HumanListStamped humanList;
    double x,y,z;
   try{
       humanList  = *(ros::topic::waitForMessage<toaster_msgs::HumanListStamped>("pdg/humanList",ros::Duration(1)));
       for(std::vector<toaster_msgs::Human>::iterator it = humanList.humanList.begin(); it != humanList.humanList.end(); it++){
         if(it->meAgent.meEntity.id == "HERAKLES_HUMAN1"){
             for(std::vector<toaster_msgs::Joint>::iterator itt = it->meAgent.skeletonJoint.begin(); itt != it->meAgent.skeletonJoint.end(); itt++){
                if(itt->meEntity.id == "head"){
                    x = itt->meEntity.pose.position.x;
                    y = itt->meEntity.pose.position.y;
                    z = itt->meEntity.pose.position.z;
                }
             }
            break;
         }
       }
   }
    catch(const std::exception & e){
        ROS_WARN("[action_executor] Failed to read human pose from toaster");
    }

    //we look at the object
    pr2motion::Head_Move_TargetGoal goal;
    goal.head_mode.value = 0;
    goal.head_target_frame = "map";
    goal.head_target_x = x;
    goal.head_target_y = y;
    goal.head_target_z = z;
    head_action_client->sendGoal(goal);



    bool finishedBeforeTimeout = head_action_client->waitForResult(ros::Duration(300.0));

    if (!finishedBeforeTimeout){
        ROS_INFO("[action_executor] pr2motion head action did not finish before the time out.");
    }
   

}
