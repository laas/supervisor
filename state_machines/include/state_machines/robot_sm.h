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
#include "supervisor_msgs/AgentList.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/Ask.h"


typedef actionlib::SimpleActionClient<supervisor_msgs::ActionExecutorAction> Client;
using namespace std;

class RobotSM{
public:
    RobotSM(ros::NodeHandle* node);
    ~RobotSM() {};
    string idleState(vector<string> partners, map<string, string> agentsState);
    string actingState();
	string waitingState();

    vector<supervisor_msgs::ActionMS> actions_;
    vector<supervisor_msgs::Action> actionsRobotReady_;
    vector<supervisor_msgs::Action> actionsXReady_;
    vector<supervisor_msgs::AgentKnowledge> knowledge_;
protected:

private:
    ros::NodeHandle* node_;
	string robotName_;
    string agentX_;
	Client actionClient_;
	bool isActing_;
    bool shouldRetractRight_;
    bool shouldRetractLeft_;
    vector<string> partners_;
    map<string, string> highLevelNames_;
    bool negociationMode_;
    double timeAdaptation_;
    bool timerStarted_;
    clock_t start_;
    vector<supervisor_msgs::ActionMS> highLevelActions_;

	void doneCb(const actionlib::SimpleClientGoalState& state, const supervisor_msgs::ActionExecutorResultConstPtr& result);
    vector<supervisor_msgs::ActionMS> getActionReady(string actor, string agent);
    vector<supervisor_msgs::ActionMS> getActionNeeded(string actor, string agent);
    supervisor_msgs::ActionMS getActionFromId(int id);
    supervisor_msgs::Action convertActionMStoAction(supervisor_msgs::ActionMS actionMS);
    bool factsAreIn(string agent, vector<toaster_msgs::Fact> facts);
    vector<supervisor_msgs::ActionMS> getIdenticalActions(vector<supervisor_msgs::ActionMS> actions);
    bool areIdentical(supervisor_msgs::ActionMS action1, supervisor_msgs::ActionMS action2);
    vector<string> getPossibleActors(supervisor_msgs::ActionMS action, string agent, map<string, string> agentsState);
    void attributeAction(supervisor_msgs::ActionMS action, string agent);
    string getHighLevelLockedObject(supervisor_msgs::ActionMS action);
    string getCorrespondingObject(supervisor_msgs::ActionMS action, string highLevelObject, string agent);
    void fillHighLevelNames();
    void initHighLevelActions();
    supervisor_msgs::ActionMS getHighLevelActionByName(string name);
    supervisor_msgs::ActionMS createActionFromHighLevel(supervisor_msgs::Action action);

};

#endif // ROBOTSM_H

