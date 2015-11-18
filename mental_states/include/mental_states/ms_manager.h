#ifndef MSMANAGER_H
#define MSMANAGER_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <mental_states/db_interface.h>
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/FactList.h"
#include "supervisor_msgs/NewPlan.h"
#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/StartGoal.h"
#include "supervisor_msgs/AbortPlan.h"
#include "supervisor_msgs/SharePlan.h"
#include "supervisor_msgs/ActionMS.h"
#include "supervisor_msgs/PlanMS.h"
#include "supervisor_msgs/GoalMS.h"
#include "supervisor_msgs/Link.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/Plan.h"


using namespace std;

class MSManager{
public:
	MSManager();
	void update(string agent);
	void initGoals();
	void initHighLevelActions();
	supervisor_msgs::GoalMS* getGoalByName(string name);
	supervisor_msgs::ActionMS createActionFromHighLevel(supervisor_msgs::Action action);
	int getAndIncreasePlanId();
	void addPlanToList(supervisor_msgs::PlanMS plan);
	pair<bool, supervisor_msgs::PlanMS> getAgentPlan(string agent);
	void abortPlan(string agent);
protected:

private:
	vector<supervisor_msgs::ActionMS> actionList;
	vector<supervisor_msgs::PlanMS> planList;
	vector<supervisor_msgs::GoalMS> goalList;
	vector<supervisor_msgs::ActionMS> highLevelActions;
	int actionId;
	int planId;
	boost::mutex actionList_mutex;
	boost::mutex planList_mutex;
	boost::mutex goalList_mutex;

	void checkEffects(string agent);
	void computePreconditions(string agent);
	void planFeasibility(string agent);
	void checkGoals(string agent);
	supervisor_msgs::ActionMS getHighLevelActionByName(string name);
	vector<supervisor_msgs::ActionMS> getActionsFromIds(vector<int> ids);

};

#endif // MSMANAGER_H

