#ifndef HUMANMONITOR_H
#define HUMANMONITOR_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/HumanPick.h"
#include "supervisor_msgs/HumanPlace.h"
#include "supervisor_msgs/ActionState.h"


using namespace std;

class HumanMonitor{
public:
	HumanMonitor();
	~HumanMonitor() {};
	void humanPick(string agent, string object);
	void humanPlace(string agent, string object, string support);
protected:

private:
	bool hasPicked;

};

#endif // HUMANMONITOR_H

