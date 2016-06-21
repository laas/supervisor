#ifndef HUMANMONITOR_H
#define HUMANMONITOR_H

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "supervisor_msgs/Action.h"
#include "supervisor_msgs/HumanActionSimu.h"
#include "supervisor_msgs/ChangeState.h"
#include "supervisor_msgs/HumanAction.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Entity.h"


using namespace std;

class HumanMonitor{
public:
	HumanMonitor() {};
	~HumanMonitor() {};
	void humanPick(string agent, string object);
	void humanPlace(string agent, string object, string support);
	void humanDrop(string agent, string object, string container);
    pair<bool, string> hasInHand(string agent);
    bool isManipulableObject(string object);
    bool isSupportObject(string support);
    bool isContainerObject(string container);
protected:

private:
    vector<pair<string, string> > attachments;

};

#endif // HUMANMONITOR_H

