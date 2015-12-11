#ifndef DBINTERFACE_H
#define DBINTERFACE_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "toaster_msgs/GetAgents.h"
#include "toaster_msgs/AddFactToAgent.h"
#include "toaster_msgs/AddFactsToAgent.h"
#include "toaster_msgs/RemoveFactToAgent.h"
#include "toaster_msgs/ExecuteSQL.h"
#include "toaster_msgs/AreInTable.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/FactList.h"
#include "supervisor_msgs/ActionMS.h"
#include "supervisor_msgs/PlanMS.h"
#include "supervisor_msgs/GoalMS.h"
#include "supervisor_msgs/Link.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/Plan.h"

using namespace std;

class DBInterface{
public:
	DBInterface() {};
	vector<string> getAgents();
	void addGoalState(supervisor_msgs::GoalMS goal, string agent, string state);
	void addPlanState(supervisor_msgs::PlanMS plan, string agent, string state);
	void addActionsState(vector<supervisor_msgs::ActionMS> actions, string agent, string state);
	vector<int> getActionsIdFromState(string agent, string state);
	bool factsAreIn(string agent, vector<toaster_msgs::Fact> facts);
	int getAgentIdPlan(string agent);
	void removeActionsState(string agent, string state);
	string getActionState(string agent, supervisor_msgs::ActionMS action);
	vector<string> getAgentGoals(string agent, string state);
	vector<string> getAgentsWhoSee(string agent);
	void addFacts(vector<toaster_msgs::Fact> facts, string agent);
	void addEffects(vector<toaster_msgs::Fact> facts, string agent);
protected:

private:


};

#endif // DBINTERFACE_H

