#ifndef VIRTUALACTION_H
#define VIRTUALACTION_H

#include "supervisor_msgs/Action.h"

#include <actionlib/server/simple_action_server.h>
#include "supervisor_msgs/FactsAreIn.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Entity.h"

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>


using namespace std;

class VirtualAction{
public:
	VirtualAction();
	~VirtualAction() {};
	virtual bool preconditions() = 0;
	virtual bool plan() = 0;
	virtual bool exec() = 0;
	virtual bool post() = 0;
protected:
   ros::NodeHandle node_;
   bool isManipulableObject(string object);
   bool isSupportObject(string support);
   bool isContainerObject(string container);
   bool ArePreconditionsChecked(vector<toaster_msgs::Fact> precs);
   void PutInHand(string object, string hand);
   void RemoveFromHand(string object);
   void PutInSupport(string object, string support);

   string robotName_;

private:

};

#endif // VIRTUALACTION_H

