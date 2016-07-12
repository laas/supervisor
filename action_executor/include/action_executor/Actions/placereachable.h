#ifndef PLACEREACHABLE_H
#define PLACEREACHABLE_H

#include "supervisor_msgs/Action.h"
#include "action_executor/virtual_action.h"

#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>


using namespace std;

class PlaceReachable: public VirtualAction{
public:
    PlaceReachable(supervisor_msgs::Action action, Connector* connector);
	virtual bool preconditions();
	virtual bool plan();
    virtual bool exec(Server* action_server);
	virtual bool post();
protected:

private:
	string support_;
    string targetAgent_;


};

#endif // PLACEREACHABLE_H

