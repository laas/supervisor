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
#include "supervisor_msgs/AgentKnowledge.h"
#include "supervisor_msgs/ActionMS.h"


typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;
using namespace std;

class RobotSM{
public:
	RobotSM();
	~RobotSM() {};
    string idleState();
    string actingState();
	string waitingState();

    vector<supervisor_msgs::ActionMS> actions_;
    vector<supervisor_msgs::AgentKnowledge> knowledge_;
protected:

private:
    ros::NodeHandle node_;
	string robotName_;
    string agentX_;
	Client actionClient_;
	bool isActing_;
    bool shouldRetractRight_;
    bool shouldRetractLeft_;

	void doneCb(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result);
    vector<supervisor_msgs::ActionMS> getActionReady(string actor, string agent);
    supervisor_msgs::ActionMS getActionFromId(int id);
    supervisor_msgs::Action convertActionMStoAction(supervisor_msgs::ActionMS actionMS);
    bool factsAreIn(string agent, vector<toaster_msgs::Fact> facts);

};

#endif // ROBOTSM_H

