#ifndef ACTIONEXECUTOR_H
#define ACTIONEXECUTOR_H

#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/Empty.h"
#include "supervisor_msgs/Focus.h"
#include "action_executor/virtual_action.h"
#include "action_executor/connector.h"
#include "action_executor/Actions/action_pkg.h"
#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread.hpp>


typedef actionlib::SimpleActionServer<supervisor_msgs::ActionExecutorAction> Server;
using namespace std;

class ActionExecutor{
public:
    ActionExecutor(string name);
protected:
	ros::NodeHandle node_;
	supervisor_msgs::ActionExecutorFeedback feedback_;
    supervisor_msgs::ActionExecutorResult result_;
    Server action_server_;

private:
	void execute(const supervisor_msgs::ActionExecutorGoalConstPtr& goal);
	VirtualAction* initializeAction(supervisor_msgs::Action action);


};

#endif // ACTIONEXECUTOR_H

