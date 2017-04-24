#ifndef CONNECTOR_H
#define CONNECTOR_H

#include "supervisor_msgs/ActionExecutorAction.h"
#include "supervisor_msgs/ActionsList.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "std_srvs/Empty.h"
#include <toaster_msgs/FactList.h>
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/PutInHand.h"
#include "toaster_msgs/RemoveFromHand.h"
#include "toaster_msgs/RobotListStamped.h"
#include "toaster_msgs/ObjectListStamped.h"
#include "toaster_msgs/SetEntityPose.h"
#include "toaster_msgs/SetInfoDB.h"

#include <gtp_ros_msgs/PlanAction.h>
#include <gtp_ros_msgs/PublishTraj.h>
#include <gtp_ros_msg/GTPTraj.h>

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
#include <fstream>

typedef actionlib::SimpleActionServer<supervisor_msgs::ActionExecutorAction> Server;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction> Client_Right_Arm;
typedef actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction> Client_Left_Arm;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction> Client_Right_Gripper;
typedef actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction> Client_Left_Gripper;

struct Connector{
    ros::NodeHandle* node_; /**< node handle pointer*/
    std::string robotName_; /**< robot name*/
    std::string robotToaster_; /**< robot toaster name*/

    bool gripperRightOpen_;/**< indicates if the right gripper is open*/
    bool gripperLeftOpen_;/**< indicates if the left gripper is open*/
    bool rightArmMoving_;/**< indicates if the right arm is moving*/
    bool leftArmMoving_;/**< indicates if the left arm is moving*/
    bool rightGripperMoving_;/**< indicates if the right gripper is moving*/
    bool leftGripperMoving_;/**< indicates if the left gripper is moving*/
    bool torsoMoving_;/**< indicates if the torso is moving*/

    bool stopOrder_;/**< order to stop the current action*/

    bool shouldUseRightHand_;/**< bool to impose the use of the right arm*/

    std::string rightArmPose_;/**< current right arm pose*/
    std::string leftArmPose_;/**< current left arm pose*/
    std::string rightArmRestPose_;/**< right arm rest pose*/
    std::string leftArmRestPose_;/**< left arm rest pose*/

    bool simu_;/**< indicates if in simu or not*/
    double waitActionServer_;/**< time to wait for an action server*/
    int nbPlanMax_;/**< nb max to try to plan*/
    bool noExec_;/**< flag true if we should not perform action execution*/
    bool noPlanning_;/**< flag true if we should not plan for action execution (can work only if noExec is also at true)*/
    std::string saveMode_;
    std::string saveFilePath_;
    std::ofstream fileSave_;

    ros::Publisher gtp_pub_; /**< publisher of gtp traj*/

    int previousId_;/**< previous gtp id*/
    int idGrasp_;/**< gtp id of the previous grasp*/
    actionlib::SimpleActionClient<gtp_ros_msgs::PlanAction>* acGTP_;/**< gtp action server*/
    actionlib::SimpleActionClient<pr2motion::InitAction>* PR2motion_init_;/**< pr2motion init action server*/
    actionlib::SimpleActionClient<pr2motion::Torso_MoveAction>* PR2motion_torso_;/**< pr2motion torso action server*/
    actionlib::SimpleActionClient<pr2motion::Arm_Right_MoveAction>* PR2motion_arm_right_;/**< pr2motion right arm action server*/
    actionlib::SimpleActionClient<pr2motion::Arm_Left_MoveAction>* PR2motion_arm_left_;/**< pr2motion left arm action server*/
    actionlib::SimpleActionClient<pr2motion::Gripper_Right_OperateAction>* PR2motion_gripper_right_;/**< pr2motion right gripper action server*/
    actionlib::SimpleActionClient<pr2motion::Gripper_Left_OperateAction>* PR2motion_gripper_left_;/**< pr2motion left gripper action server*/

    ros::ServiceClient client_db_execute_; /**< toaster client for database execute commands*/
    ros::ServiceClient client_put_hand_; /**< toaster client to put objects in hand*/
    ros::ServiceClient client_remove_hand_; /**< toaster client to remove objects from hand*/
    ros::ServiceClient client_set_pose_; /**< toaster client to set entities poses*/
    ros::ServiceClient client_gtp_traj_; /**< gtp client to publish a traj*/
    ros::ServiceClient client_db_set_; /**< toaster client for database set info commands*/

    double timeDB_;/**< time spent in the database*/
    double timePlan_;/**< time spent in the planning*/
    double timeExec_;/**< time spent in the execution*/
    double timeGTP_;/**< time spent in other gtp actions*/
    double timeToaster_;/**< time spent in other gtp actions*/
    ros::Time timerStart_;/**< timer set at the begining of the action*/

    bool isActing_; /**< Flag indicating if the robot is executing an action*/
    supervisor_msgs::Action currentAction_; /**< Current action executed by the robot*/

    std::map<std::string, std::string> highLevelNames_; /**< high level names of objects*/
    std::map<std::string, std::vector<std::string> > highLevelRefinment_; /**< possible refinment for high level names*/
    std::string humanCost_; /**< human in which one we apply the cost*/
    std::map<std::string, double> humanDistances_; /**< distance between objects and the chosen human*/
    std::string objectToWatch_; /**< object to monitor*/
    bool refineOrder_; /**< flag to indicate we should look for an other refinment for the object to watch*/

};

#endif // CONNECTOR_H
