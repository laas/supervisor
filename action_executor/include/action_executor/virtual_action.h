#ifndef VIRTUALACTION_H
#define VIRTUALACTION_H

#include "supervisor_msgs/Action.h"

#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>


using namespace std;

class VirtualAction{
public:
	VirtualAction() {};
	~VirtualAction() {};
	virtual bool preconditions() = 0;
	virtual bool plan() = 0;
	virtual bool exec() = 0;
	virtual bool post() = 0;
protected:

private:


};

#endif // VIRTUALACTION_H

