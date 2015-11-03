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
#include "supervisor_msgs/ActionMS.h"
#include "supervisor_msgs/PlanMS.h"
#include "supervisor_msgs/GoalMS.h"
#include "supervisor_msgs/Link.h"


using namespace std;

class MSManager{
public:
	MSManager() {};
	void update(string agent);
protected:

private:
	vector<supervisor_msgs::ActionMS> actionList;
	vector<supervisor_msgs::PlanMS> planList;
	vector<supervisor_msgs::GoalMS> goalList;
	int actionId;
	int planId;
	boost::mutex actionList_mutex;
	boost::mutex planList_mutex;
	boost::mutex goalList_mutex;

};

#endif // MSMANAGER_H

