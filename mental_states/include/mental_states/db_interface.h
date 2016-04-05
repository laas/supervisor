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
#include "toaster_msgs/GetFacts.h"
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
    void addFactToAdd(toaster_msgs::Fact fact, string agent);
    void addFactToRemove(toaster_msgs::Fact fact, string agent);
	void addGoalState(supervisor_msgs::GoalMS goal, string agent, string state);
	void addPlanState(supervisor_msgs::PlanMS plan, string agent, string state);
	void addActionsState(vector<supervisor_msgs::ActionMS> actions, string agent, string state);
	vector<int> getActionsIdFromState(string agent, string state);
	bool factsAreIn(string agent, vector<toaster_msgs::Fact> facts);
	int getAgentIdPlan(string agent);
	void removeActionsState(string agent, string state);
	string getActionState(string agent, supervisor_msgs::ActionMS action);
	string getGoalState(string agent, supervisor_msgs::GoalMS goal);
    string getPlanState(string agent, supervisor_msgs::PlanMS plan);
	vector<string> getAgentGoals(string agent, string state);
	vector<string> getAgentsWhoSee(string agent);
    bool isAgentSeeing(string agentTested, string agentToSee);
	void addFacts(vector<toaster_msgs::Fact> facts, string agent);
    void removeFacts(vector<toaster_msgs::Fact> facts, string agent);
	void addEffects(vector<toaster_msgs::Fact> facts, string agent);
	vector<toaster_msgs::Fact> getFactsAgent(string agent);
	void cleanDB();
    void updateKnowledge();
    void initKnowledge(vector<string> agents);
protected:

private:
    vector<pair<string, vector<toaster_msgs::Fact> > > knowledge_;
    vector<pair<string, vector<toaster_msgs::Fact> > > toAdd_;
    vector<pair<string, vector<toaster_msgs::Fact> > > toRemove_;


};

#endif // DBINTERFACE_H

