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
protected:

private:


};

#endif // DBINTERFACE_H

