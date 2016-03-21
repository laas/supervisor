#ifndef MOVETO_H
#define MOVETO_H

#include "supervisor_msgs/Action.h"
#include "action_executor/virtual_action.h"

#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>


using namespace std;

class MoveTo: public VirtualAction{
public:
	MoveTo(supervisor_msgs::Action action);
	virtual bool preconditions();
	virtual bool plan();
	virtual bool exec();
	virtual bool post();
protected:

private:
	string position_;


};

#endif // MOVETO_H

