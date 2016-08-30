#ifndef PICKANDDROP_H
#define PICKANDDROP_H

#include "supervisor_msgs/Action.h"
#include "action_executor/virtual_action.h"

#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>


using namespace std;

class PickAndDrop: public VirtualAction{
public:
	PickAndDrop(supervisor_msgs::Action action, Connector* connector);
	virtual bool preconditions();
	virtual bool plan();
    virtual bool exec(Server* action_server);
	virtual bool post();
    virtual supervisor_msgs::Action getInstantiatedAction();
protected:

private:
	string container_;
    bool containerRefined_;
    int nextActionId_;


};

#endif // PICKANDDROP_H

