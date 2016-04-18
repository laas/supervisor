#ifndef VIRTUALACTION_H
#define VIRTUALACTION_H

#include "supervisor_msgs/Action.h"
#include "action_executor/connector.h"

#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/GetInfo.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/RobotList.h"
#include "toaster_msgs/Robot.h"
#include "toaster_msgs/Entity.h"
#include <gtp_ros_msg/requestAction.h>
#include <gtp_ros_msg/Req.h>
#include <gtp_ros_msg/Ag.h>
#include <gtp_ros_msg/Obj.h>
#include <gtp_ros_msg/Iden.h>
#include <pr2motion/Arm_Right_MoveAction.h>
#include <pr2motion/Arm_Left_MoveAction.h>
#include <pr2motion/Gripper_Right_OperateAction.h>
#include <pr2motion/Gripper_Left_OperateAction.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


typedef actionlib::SimpleActionServer<supervisor_msgs::ActionExecutorAction> Server;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction> Client_Right_Arm;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction> Client_Left_Arm;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction> Client_Right_Gripper;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction> Client_Left_Gripper;
using namespace std;


class VirtualAction{
public:
	VirtualAction(Connector* connector);
	~VirtualAction() {};
	virtual bool preconditions() = 0;
	virtual bool plan() = 0;
    virtual bool exec(Server* action_server) = 0;
	virtual bool post() = 0;

    void moveRightArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Right_MoveResultConstPtr& result);
    void moveLeftArm(const actionlib::SimpleClientGoalState& state, const pr2motion::Arm_Left_MoveResultConstPtr& result);
    void moveRightGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Right_OperateResultConstPtr& result);
    void moveLeftGripper(const actionlib::SimpleClientGoalState& state, const pr2motion::Gripper_Left_OperateResultConstPtr& result);
protected:
   ros::NodeHandle node_;
   bool isManipulableObject(string object);
   bool isSupportObject(string support);
   bool isContainerObject(string container);
   bool ArePreconditionsChecked(vector<toaster_msgs::Fact> precs);
   void PutInHand(string object, string hand, int gtpId);
   void RemoveFromHand(string object);
   void PutOnSupport(string object, string support);
   void PutInContainer(string object, string container);
   bool updateGTP();
   int  planGTP(string actionName, vector<gtp_ros_msg::Ag> agents, vector<gtp_ros_msg::Obj> objects, vector<gtp_ros_msg::Data> datas, vector<gtp_ros_msg::Points> points);
   bool execAction(int actionId, bool shouldOpen, Server* action_server);
   bool executeTrajectory(int actionId, int actionSubId, int armId, Server* action_server);
   bool openGripper(int armId, Server* action_server);
   bool closeGripper(int armId, Server* action_server);
   bool isGripperEmpty(string arm);

   string object_;
   int actionId_;
   string robotName_;
   double waitActionServer_;
   Connector* connector_;
   bool simu_;
   int nbPlanMax_;
   bool gripperEmpty_;

private:

};

#endif // VIRTUALACTION_H

