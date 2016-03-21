#ifndef CONNECTOR_H
#define CONNECTOR_H


#include <gtp_ros_msg/requestAction.h>
#include <pr2motion/Arm_Right_MoveAction.h>
#include <pr2motion/InitAction.h>
#include <pr2motion/connect_port.h>
#include <pr2motion/Torso_MoveAction.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


using namespace std;

class Connector{
public:
	Connector();
	~Connector() {};
	
	int previousId_;
   actionlib::SimpleActionClient<gtp_ros_msg::requestAction>* acGTP_;
   actionlib::SimpleActionClient<pr2motion::InitAction>* PR2motion_init_;
   actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>* PR2motion_torso_;
   
protected:

private:
   ros::NodeHandle node_;
   double waitActionServer_;
	bool simu_;

};

#endif // CONNECTOR_H

