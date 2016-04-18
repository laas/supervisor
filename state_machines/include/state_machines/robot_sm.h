#ifndef ROBOTSM_H
#define ROBOTSM_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "supervisor_msgs/GetInfo.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ActionExecutorActionResult.h"
#include "supervisor_msgs/ActionExecutorActionFeedback.h"


typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;
using namespace std;

class RobotSM{
public:
	RobotSM();
	~RobotSM() {};
	string idleState();
	string actingState();
	string waitingState();
protected:

private:
    ros::NodeHandle node_;
	string robotName_;
	Client actionClient_;
	bool isActing_;

	void doneCb(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result);

};

#endif // ROBOTSM_H

