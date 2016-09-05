#ifndef CONNECTOR_H
#define CONNECTOR_H


#include "supervisor_msgs/ActionExecutorAction.h"
#include <gtp_ros_msg/requestAction.h>
#include <pr2motion/Arm_Right_MoveAction.h>
#include <pr2motion/Arm_Left_MoveAction.h>
#include <pr2motion/Gripper_Right_OperateAction.h>
#include <pr2motion/Gripper_Left_OperateAction.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Torso_MoveAction.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


using namespace std;

class Connector{
public:
	Connector();
	~Connector() {};

    bool gripperRightOpen_;
    bool gripperLeftOpen_;
    bool rightArmMoving_;
    bool leftArmMoving_;
    bool rightGripperMoving_;
    bool leftGripperMoving_;
    bool torsoMoving_;

    bool stopOrder_;

    bool shouldUseRightHand_;

    string rightArmPose_;
    string leftArmPose_;
    string rightArmRestPose_;
    string leftArmRestPose_;

    string objectFocus_;
    double weightFocus_;
    bool stopableFocus_;
    bool simu_;

    map<string, double> robotDistances_;
    map<string, double> humanDistances_;

	
    int previousId_;
    int idGrasp_;
    actionlib::SimpleActionClient<gtp_ros_msg::requestAction>* acGTP_;
    actionlib::SimpleActionClient<pr2motion::InitAction>* PR2motion_init_;
    actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>* PR2motion_torso_;
    actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>* PR2motion_arm_right_;
    actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>* PR2motion_arm_left_;
    actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>* PR2motion_gripper_right_;
    actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>* PR2motion_gripper_left_;
   
protected:

private:
   ros::NodeHandle node_;
   double waitActionServer_;

};

#endif // CONNECTOR_H

