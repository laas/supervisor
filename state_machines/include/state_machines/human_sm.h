#ifndef HUMAN_H
#define HUMAN_H

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
#include "supervisor_msgs/SolveDivergentBelief.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ActionExecutorActionResult.h"
#include "supervisor_msgs/ActionExecutorActionFeedback.h"
#include <toaster_msgs/Fact.h>
#include <toaster_msgs/FactList.h>


using namespace std;

class HumanSM{
public:
	HumanSM(string humanName);
	~HumanSM() {};
	string idleState();
    string actingState(vector<string>* objects, bool* unexpected);
	string waitingState();
    string shouldActState(string robotState, vector<string>* objects);
	string absentState();
    void areaFactListCallback(const toaster_msgs::FactList::ConstPtr& msg);
    vector<string> focusObjects(supervisor_msgs::Action action, bool shouldAct);

    bool humanActs_;
    supervisor_msgs::Action shouldDoAction_;
    supervisor_msgs::Action PerformedAction_;
protected:

private:
	ros::NodeHandle node_;
	string humanName_;
	string robotName_;
	bool simu_;
	double timeToWait_;
	bool timerStarted_;
	clock_t start_;
    bool present_;


    ros::Subscriber subArea_;

};

#endif // HUMAN_H

