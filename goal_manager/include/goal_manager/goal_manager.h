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
#include "supervisor_msgs/StartGoal.h"

using namespace std;

class GoalManager {
public:
	GoalManager();
	~GoalManager() {};
	void addGoal(string goal);
protected:

private:
   string currentGoal_;
   queue<string> waitingGoals_; 
	void executeGoal(string goal);

};

#endif // GOALMANAGER_H

