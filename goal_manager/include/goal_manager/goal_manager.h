#ifndef GOALMANAGER_H
#define GOALMANAGER_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "supervisor_msgs/NewGoal.h"
#include "supervisor_msgs/EndPlan.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/Plan.h"
#include "hatp_msgs/PlanningRequest.h"
#include "hatp_msgs/Plan.h"
#include "hatp_msgs/Request.h"
#include "hatp_msgs/StreamNode.h"
#include "hatp_msgs/Task.h"

using namespace std;

class GoalManager {
public:
	GoalManager();
	~GoalManager() {};
	void addGoal(string goal);
	void endPlan(bool report);
	void chooseGoal();
protected:

private:
   string currentGoal_;
   bool hasPlan_;
   queue<string> waitingGoals_; 
	void executeGoal(string goal);
	supervisor_msgs::Plan convertPlan(hatp_msgs::Plan plan, string goal);

};

#endif // GOALMANAGER_H

