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
    GoalManager(ros::NodeHandle* node);
	~GoalManager() {};
	void addGoal(string goal);
    void endGoal(bool report);
	void chooseGoal();
protected:

private:
   string currentGoal_;
   queue<string> waitingGoals_;
   ros::NodeHandle* node_;

};

#endif // GOALMANAGER_H

