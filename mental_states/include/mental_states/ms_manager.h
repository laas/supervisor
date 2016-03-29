#ifndef MSMANAGER_H
#define MSMANAGER_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread.hpp>

#include <mental_states/db_interface.h>
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/FactList.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/InfoGiven.h"
#include "supervisor_msgs/GetInfo.h"
#include "supervisor_msgs/SolveDivergentBelief.h"
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
	vector<supervisor_msgs::ActionMS> getActionList();
    void update(string agent);
	void initGoals();
	void initHighLevelActions();
	supervisor_msgs::GoalMS* getGoalByName(string name);
	supervisor_msgs::ActionMS createActionFromHighLevel(supervisor_msgs::Action action);
	int getAndIncreasePlanId();
	void addPlanToList(supervisor_msgs::PlanMS plan);
	pair<bool, supervisor_msgs::PlanMS> getAgentPlan(string agent);
	void abortPlan(string agent);
	pair<bool, supervisor_msgs::ActionMS> getActionFromAction(supervisor_msgs::Action action);
	pair<bool, supervisor_msgs::ActionMS> getActionFromId(int id);
	pair<bool, supervisor_msgs::PlanMS> getPlanFromId(int id);
	supervisor_msgs::Action convertActionMStoAction(supervisor_msgs::ActionMS actionMS);
	vector<supervisor_msgs::ActionMS> getActionsFromIds(vector<int> ids);

    DBInterface db_;
protected:

private:
	vector<supervisor_msgs::ActionMS> actionList_;
	vector<supervisor_msgs::PlanMS> planList_;
	vector<supervisor_msgs::GoalMS> goalList_;
	vector<supervisor_msgs::ActionMS> highLevelActions_;
	int actionId_;
	int planId_;
	boost::mutex actionListMutex_;
	boost::mutex planListMutex_;
	boost::mutex goalListMutex_;

	void checkEffects(string agent);
	void computePreconditions(string agent);
	void planFeasibility(string agent);
	void checkGoals(string agent);
	supervisor_msgs::ActionMS getHighLevelActionByName(string name);

};

#endif // MSMANAGER_H

