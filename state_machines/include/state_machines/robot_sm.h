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

#include "supervisor_msgs/GetActionTodo.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/ActionExecutorAction.h"


typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;
using namespace std;

class RobotSM{
public:
	RobotSM();
	~RobotSM() {};
	string idleState(string humanState);
	string actingState(string humanState);
	string waitingState(string humanState);
protected:

private:
	ros::NodeHandle node;
	string robot_name;
	Client action_client;

};

#endif // ROBOTSM_H

