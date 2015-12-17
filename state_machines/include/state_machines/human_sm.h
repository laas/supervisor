#ifndef HUMAN_H
#define HUMAN_H

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
#include "supervisor_msgs/GetActionState.h"
#include "supervisor_msgs/SolveDivergentBelief.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ActionExecutorActionResult.h"
#include "supervisor_msgs/ActionExecutorActionFeedback.h"


typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;
using namespace std;

class HumanSM{
public:
	HumanSM(string _human_name);
	~HumanSM() {};
	string idleState();
	string actingState();
	string waitingState();
	string shouldActState();
	string absentState();
protected:

private:
	ros::NodeHandle node;
	string human_name;
	string robot_name;
	Client action_client;

};

#endif // HUMAN_H

