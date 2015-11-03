#ifndef DBINTERFACE_H
#define DBINTERFACE_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

#include "toaster_msgs/GetAgents.h"

using namespace std;

class DBInterface{
public:
	DBInterface() {};
	vector<string> getAgents();
protected:
	ros::NodeHandle node;
private:


};

#endif // DBINTERFACE_H

