#ifndef VIRTUALACTION_H
#define VIRTUALACTION_H

#include "supervisor_msgs/Action.h"
#include "action_executor/connector.h"

#include <actionlib/server/simple_action_server.h>
#include "supervisor_msgs/FactsAreIn.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Entity.h"
#include <gtp_ros_msg/requestAction.h>
#include <gtp_ros_msg/Req.h>
#include <gtp_ros_msg/Ag.h>
#include <gtp_ros_msg/Obj.h>
#include <gtp_ros_msg/Iden.h>
#include <pr2motion/Arm_Right_MoveAction.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


using namespace std;

class VirtualAction{
public:
	VirtualAction(Connector* connector);
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
   bool updateGTP();
   int  planGTP(string actionName, vector<gtp_ros_msg::Ag> agents, vector<gtp_ros_msg::Obj> objects, vector<gtp_ros_msg::Data> datas, vector<gtp_ros_msg::Points> points);
   bool execAction(int actionId, bool shouldOpen);
   bool executeTrajectory(int actionId, int actionSubId, int armId);
   bool openGripper(int armId);
   bool closeGripper(int armId);

   string object_;
   int actionId_;
   string robotName_;
   double waitActionServer_;
   Connector* connector_;
   bool simu_;
   int nbPlanMax_;

private:

};

#endif // VIRTUALACTION_H

