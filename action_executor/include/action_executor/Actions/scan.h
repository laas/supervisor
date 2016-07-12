#ifndef SCAN_H
#define SCAN_H

#include "supervisor_msgs/Action.h"
#include "action_executor/virtual_action.h"

#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>


using namespace std;

class Scan: public VirtualAction{
public:
    Scan(supervisor_msgs::Action action, Connector* connector);
	virtual bool preconditions();
	virtual bool plan();
    virtual bool exec(Server* action_server);
	virtual bool post();
protected:

private:
    double timeScan_;
    clock_t start_;

    void controlRobotLight(bool on);


};

#endif // SCAN_H
