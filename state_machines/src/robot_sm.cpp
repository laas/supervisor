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
    ros::Publisher tag_detection_pub = node_.advertise <std_msgs::Bool>("ar_track_alvar/enable_detection",1);
    std_msgs::Bool msg;
    msg.data  = false;
    tag_detection_pub.publish(msg);
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
